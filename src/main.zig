const std = @import("std");

const expect = std.testing.expect;

// Virtual Static Memory
pub const Memory = struct {
    const Self = @This();
    allocator: std.mem.Allocator,
    raw_memory: []u8,

    pub fn init(allocator: std.mem.Allocator, size: usize) !*Memory {
        var temp = try allocator.create(Self);
        temp.allocator = allocator;
        temp.raw_memory = try allocator.alloc(u8, size);
        for (0..size) |i| {
            temp.raw_memory[i] = 0;
        }
        return temp;
    }

    pub fn deinit(self: *Self) void {
        self.allocator.free(self.raw_memory);
        self.allocator.destroy(self);
    }

    // Read Byte at address
    pub fn readByte(self: *Self, address: u32) !u8 {
        if (address >= self.raw_memory.len) return error.AddressOutOfRange;
        return self.raw_memory[address];
    }
    // Read Halfword at address
    pub fn readHalfword(self: *Self, address: u32) !u16 {
        if (address >= self.raw_memory.len - 1) return error.AddressOutOfRange;
        const p: *[2]u8 = @ptrCast(self.raw_memory.ptr + address);
        return @bitCast(p.*);
    }
    // Read Word at address
    pub fn readWord(self: *Self, address: u32) !u32 {
        if (address >= self.raw_memory.len - 3) return error.AddressOutOfRange;
        const p: *[4]u8 = @ptrCast(self.raw_memory.ptr + address);
        return @bitCast(p.*);
    }

    // Write Byte at address
    pub fn writeByte(self: *Self, address: u32, value: u8) !void {
        if (address >= self.raw_memory.len) return error.AddressOutOfRange;
        self.raw_memory[address] = value;
    }
    // Write Halfword at address
    pub fn writeHalfword(self: *Self, address: u32, value: u16) !void {
        if (address >= self.raw_memory.len - 1) return error.AddressOutOfRange;
        const p: *[2]u8 = @ptrCast(self.raw_memory.ptr + address);
        p.* = @bitCast(value);
    }
    // Write Word at address
    pub fn writeWord(self: *Self, address: u32, value: u32) !void {
        if (address >= self.raw_memory.len - 3) return error.AddressOutOfRange;
        const p: *[4]u8 = @ptrCast(self.raw_memory.ptr + address);
        p.* = @bitCast(value);
    }
};
test "Memory" {
    const allocator = std.testing.allocator;
    const mem = try Memory.init(allocator, 1024);
    try mem.writeHalfword(0x10, 259);
    try mem.writeByte(0x13, 4);
    std.debug.print("Byte at 0x10:\t   0b{0b:0>8}({0d})\n", .{try mem.readByte(0x10)});
    std.debug.print("Byte at 0x11:\t   0b{0b:0>8}({0d})\n", .{try mem.readByte(0x11)});
    std.debug.print("Byte at 0x12:\t   0b{0b:0>8}({0d})\n", .{try mem.readByte(0x12)});
    std.debug.print("Byte at 0x13:\t   0b{0b:0>8}({0d})\n", .{try mem.readByte(0x13)});
    std.debug.print("Halfword at 0x10:  0b{0b:0>16}({0d})\n", .{try mem.readHalfword(0x10)});
    std.debug.print("Word at 0x10:\t   0b{0b:0>32}({0d})\n", .{try mem.readWord(0x10)});
    mem.deinit();
}

pub const RegisterFile = struct {
    const Self = @This();
    allocator: std.mem.Allocator,
    registers: []i32,

    pub fn read(self: *Self, id: u8) i32 {
        if (id > 31) return 0;
        if (id == 0) return 0;
        return self.registers[id - 1];
    }
    pub fn write(self: *Self, id: u8, value: i32) void {
        if (id > 31) return;
        if (id == 0) return;
        self.registers[id - 1] = value;
    }

    pub fn init(allocator: std.mem.Allocator) !Self {
        const temp = Self{
            .allocator = allocator,
            .registers = try allocator.alloc(i32, 31),
        };

        for (temp.registers) |*v| {
            v.* = 0;
        }

        return temp;
    }

    pub fn deinit(self: *Self) void {
        self.allocator.free(self.registers);
    }
};

const Branch = enum(u8) {
    NONE = 0b00,
    ONE = 0b01,
    ZERO = 0b10,
};
const ControlSignals = struct {
    result_src: u2 = 0b00,
    mem_write: u1 = 0b0,
    alu_op: u2 = 0b00,
    alu_src: u1 = 0b0,
    imm_src: u2 = 0b00,
    reg_write: u1 = 0b0,
    branch: u8 = 0b0,
    jump: u1 = 0b0,
};

pub const ALU_CONTROL = enum(u8) {
    ADD = 0b000,
    SUBTRACT = 0b001,
    AND = 0b010,
    OR = 0b011,
    SET_LESS_THAN = 0b101,
};

pub const ControlUnit = struct {
    const Self = @This();
    allocator: std.mem.Allocator,
    instruction_map: std.AutoHashMap(u32, ControlSignals),
    control_signals: ControlSignals,

    fn parseInstructionSet(self: *Self, path: []const u8) !void {
        const input_file = try std.fs.cwd().openFile(path, .{});
        defer input_file.close();

        const file_stat = try input_file.stat();
        // defer self.allocator.destroy(file_stat);

        const input = try input_file.readToEndAllocOptions(self.allocator, file_stat.size, null, std.mem.Alignment.@"8", 0);
        std.debug.print("input size: {d}\ninput: {s}\n", .{ input.len, input });
        defer self.allocator.free(input);

        var status: std.zon.parse.Diagnostics = .{};
        defer status.deinit(self.allocator);
        const parsed = try std.zon.parse.fromSlice(struct { instructions: []struct {
            op: u32,
            control_signals: ControlSignals,
        } }, self.allocator, input, &status, .{ .free_on_error = true });
        defer self.allocator.free(parsed.instructions);
        // const zon: struct { instructions: []struct {
        //     op: u32,
        //     control_signals: struct {
        //         result_src: u2,
        //         mem_write: u1,
        //         alu_control: u3,
        //         alu_src: u1,
        //         imm_src: u2,
        //         reg_write: u1,
        //         branch: u8,
        //     },
        // } } = @import("instructions.zon");

        for (parsed.instructions) |entry| {
            std.debug.print("INSTRUCTION ENTRY 0b{0b:0>7}: {1any}\n", .{ entry.op, entry.control_signals });
            try self.instruction_map.put(entry.op, entry.control_signals);
        }
    }

    pub fn init(allocator: std.mem.Allocator) !*Self {
        const self = try allocator.create(Self);
        self.allocator = allocator;
        self.instruction_map = std.AutoHashMap(u32, ControlSignals).init(allocator);
        self.control_signals = ControlSignals{};
        try parseInstructionSet(self, "src/instructions.zon");
        return self;
    }

    pub fn deinit(self: *Self) void {
        self.instruction_map.deinit();
        self.allocator.destroy(self);
    }

    pub fn decodeMain(self: *Self, op: u32) !ControlSignals {
        if (self.instruction_map.get(op)) |v| {
            self.control_signals = v;
            return v;
        } else return error.IllegalInstruction;
    }

    // Decodes ALU, returns ADD for illegal codes
    pub fn decodeALU(self: *Self, funct3: u32, funct7: u32, op: u32) ALU_CONTROL {
        switch (self.control_signals.alu_op) {
            0b00 => return ALU_CONTROL.ADD,
            0b01 => return ALU_CONTROL.SUBTRACT,
            0b10 => {
                switch (funct3) {
                    0b000 => {
                        const t = ((funct7 | op) >> 4) & 0b11;
                        // std.debug.print("{0b:0>2}\n", .{t});
                        switch (t) {
                            0b00, 0b01, 0b10 => return ALU_CONTROL.ADD,
                            0b11 => return ALU_CONTROL.SUBTRACT,
                            else => return ALU_CONTROL.ADD,
                            // else => return error.IllegalInstruction,
                        }
                    },
                    0b010 => return ALU_CONTROL.SET_LESS_THAN,
                    0b110 => return ALU_CONTROL.OR,
                    0b111 => return ALU_CONTROL.AND,
                    else => return ALU_CONTROL.ADD,
                    // else => error.IllegalInstruction,
                }
            },
            else => return ALU_CONTROL.ADD,
            // 0b11 => return error.IllegalInstruction,
        }
    }
};

pub const ALU = struct {
    const Self = @This();

    mode: ALU_CONTROL = ALU_CONTROL.ADD,
    result: i32 = 0,
    zero: bool = false,

    pub fn setMode(self: *Self, mode: ALU_CONTROL) void {
        self.mode = mode;
    }

    pub fn calculate(self: *Self, a: i32, b: i32) void {
        var res: i32 = 0;
        switch (self.mode) {
            ALU_CONTROL.ADD => {
                res = a + b;
            },
            ALU_CONTROL.SUBTRACT => {
                res = a - b;
            },
            ALU_CONTROL.AND => {
                res = a & b;
            },
            ALU_CONTROL.OR => {
                res = a | b;
            },
            ALU_CONTROL.SET_LESS_THAN => {
                res = @intFromBool(a < b);
            },
        }
        self.zero = res == 0;
        self.result = res;
    }

    pub fn getResult(self: *Self) i32 {
        return self.result;
    }

    pub fn getZero(self: *Self) bool {
        return self.zero;
    }
};

test ALU {
    var alu = ALU{};
    alu.setMode(ALU_CONTROL.ADD);
    alu.calculate(1, 3);
    try expect(alu.result == 4);
    alu.setMode(ALU_CONTROL.SUBTRACT);
    alu.calculate(4, 1);
    try expect(alu.result == 3);
    alu.setMode(ALU_CONTROL.AND);
    alu.calculate(3, 2);
    try expect(alu.result == 2);
    alu.setMode(ALU_CONTROL.OR);
    alu.calculate(4, 5);
    try expect(alu.result == 5);
    alu.setMode(ALU_CONTROL.SET_LESS_THAN);
    alu.calculate(3, 9);
    try expect(alu.result == 1);
}

pub const Extend = struct {
    const Self = @This();
    src: u2,

    pub fn extend(self: *Self, instr: u32) i32 {
        var temp: u32 = 0;
        switch (self.src) {
            // I-Type
            0b00 => {
                temp |= (instr >> 20) & 0b111111111111;
            },
            0b01 => {
                temp |= (instr >> 7) & 0b11111;
                temp |= ((instr >> 25) & 0b1111111) << 5;
            },
            0b10 => {
                temp |= ((instr >> 8) & 0b1111) << 1;
                temp |= ((instr >> 25) & 0b111111) << 5;
                temp |= ((instr >> 7) & 0b1) << 11;
                temp |= ((instr >> 31) & 0b1) << 12;
            },
            0b11 => {
                temp |= ((instr >> 21) & 0b1111111111) << 1;
                temp |= ((instr >> 20) & 0b1) << 11;
                temp |= ((instr >> 12) & 0b11111111) << 12;
                temp |= ((instr >> 31) & 0b1) << 20;
            },
        }
        return @bitCast(temp);
    }
};

pub const Model = struct {
    const Self = @This();
    allocator: std.mem.Allocator,
    instruction_memory: *Memory,
    data_memory: *Memory,
    register_file: RegisterFile,
    control_unit: *ControlUnit,
    alu: ALU,
    extend: Extend,
    pc: u32,

    pub fn init(allocator: std.mem.Allocator, mem_size: u32, instruction_memory: *Memory) !*Self {
        var self = try allocator.create(Self);
        self.allocator = allocator;
        self.instruction_memory = instruction_memory;
        self.data_memory = try Memory.init(allocator, mem_size);
        self.register_file = try RegisterFile.init(allocator);
        self.control_unit = try ControlUnit.init(allocator);
        self.alu = ALU{};
        self.extend = Extend{ .src = 0 };
        self.pc = 0x0;
        return self;
    }

    pub fn deinit(self: *Self) void {
        self.instruction_memory.deinit();
        self.data_memory.deinit();
        self.register_file.deinit();
        self.control_unit.deinit();
        self.allocator.destroy(self);
    }

    // fn replaceIllegalWithNOP(self: *Self, ) ControlSignals

    pub fn tick(self: *Self) !void {
        const instr = try self.instruction_memory.readWord(self.pc);
        const op: u32 = instr & 0b1111111;
        const funct3: u32 = (instr >> 12) & 0b111;
        const funct7: u32 = (instr >> 25) & 0b1111111;
        const a1: u32 = (instr >> 15) & 0b11111;
        const a2: u32 = (instr >> 20) & 0b11111;
        const a3: u8 = @truncate((instr >> 7) & 0b11111);
        std.debug.print("Current Instruction 0x{1x:0>8}: 0b{0b:0>32} (op: 0b{2b:0>7}, funct3: 0b{3b:0>3}, funct7: 0b{4b:0>7}, a1: 0b{5b:0>5}, a2: 0b{6b:0>5}, a3: 0b{7b:0>5})\n", .{ try self.instruction_memory.readWord(self.pc), self.pc, op, funct3, funct7, a1, a2, a3 });

        const rd1: i32 = self.register_file.read(@truncate(a1));
        const rd2: i32 = self.register_file.read(@truncate(a2));

        const control_signals: ControlSignals = try self.control_unit.decodeMain(op);
        std.debug.print("Control Signals: {any}\n", .{control_signals});
        const alu_control: ALU_CONTROL = self.control_unit.decodeALU(funct3, funct7, op);

        self.alu.setMode(alu_control);
        const src_a = rd1;
        self.extend.src = control_signals.imm_src;
        const src_b = switch (control_signals.alu_src) {
            0b0 => rd2,
            0b1 => self.extend.extend(instr),
        };
        self.alu.calculate(src_a, src_b);
        const alu_res = self.alu.getResult();
        std.debug.print("ALU operation ({any}): [{d}, {d}] = {d}\n", .{ alu_control, src_a, src_b, alu_res });

        if (@bitCast(control_signals.reg_write)) {
            self.register_file.write(a3, alu_res);
            std.debug.print("Saved {d} to register x{:0>1}\n", .{ self.register_file.read(a3), a3 });
        }

        if (@bitCast(control_signals.mem_write)) {
            // self.data_memory.writeWord(, value: u32)
        }

        self.pc += 4;
    }
};

test Model {
    const allocator = std.testing.allocator;
    const instruction_memory = try Memory.init(allocator, 1024);
    const model = try Model.init(allocator, 1024, instruction_memory);
    defer model.deinit();

    try instruction_memory.writeWord(0x00, 0b000000001011_00000_110_00101_0010011); // ori t0, x0, 11
    try instruction_memory.writeWord(0x04, 0b000000000001_00000_000_00110_0010011); // addi t1, x0, 1
    try instruction_memory.writeWord(0x08, 0b0100000_00110_00101_000_00101_0110011); // sub t0, t0, t1

    model.tick() catch |err| {
        std.debug.print("UNEXPECTED ERROR OCCURED: {}\n", .{err});
    };
    model.tick() catch |err| {
        std.debug.print("UNEXPECTED ERROR OCCURED: {}\n", .{err});
    };
    model.tick() catch |err| {
        std.debug.print("UNEXPECTED ERROR OCCURED: {}\n", .{err});
    };
}

pub fn main() void {
    std.debug.print("Hello Zig!\n", .{});
    const allocator = std.heap.ArenaAllocator.init(std.heap.page_allocator).allocator();
    const mem = try Memory.init(allocator, 1024);
    defer mem.deinit();
}
