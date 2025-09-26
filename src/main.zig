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
        return Self{
            .allocator = allocator,
            .registers = try allocator.alloc(i32, 31),
        };
    }

    pub fn deinit(self: *Self) void {
        self.allocator.free(self.registers);
    }
};

const Branch = enum(u8) {
    NONE,
    ONE,
    ZERO,
};
const ControlSignals = struct {
    result_src: u2 = 0b00,
    mem_write: u1 = 0b0,
    alu_control: u3 = 0b000,
    alu_src: u1 = 0b0,
    imm_src: u2 = 0b00,
    reg_write: u1 = 0b0,
    branch: u8 = 0b0,
};

pub const ControlUnit = struct {
    const Self = @This();
    allocator: std.mem.Allocator,
    instruction_map: std.AutoHashMap(u32, ControlSignals),

    fn parseInstructionSet(self: *Self, path: []const u8) !void {
        const input_file = try std.fs.cwd().openFile(path, .{});
        defer input_file.close();

        const file_stat = try input_file.stat();
        // defer self.allocator.destroy(file_stat);

        const input = try input_file.readToEndAllocOptions(self.allocator, file_stat.size, null, @sizeOf(u8), 0);
        std.debug.print("input size: {d}\ninput: {s}\n", .{ input.len, input });
        defer self.allocator.free(input);

        var status: std.zon.parse.Status = .{};
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
        try parseInstructionSet(self, "src/instructions.zon");
        return self;
    }

    pub fn deinit(self: *Self) void {
        self.instruction_map.deinit();
        self.allocator.destroy(self);
    }

    pub fn decode(self: *Self, op: u32) !ControlSignals {
        if (self.instruction_map.get(op)) |v| {
            return v;
        } else return error.IllegalInstruction;
    }
};

pub const ALU = struct {
    const Self = @This();
    pub const ALU_MODE = enum(u8) {
        ADD,
        SUBTRACT,
        AND,
        OR,
        SET_LESS_THAN,
    };

    mode: ALU_MODE = ALU_MODE.ADD,

    pub fn setMode(self: *Self, mode: ALU_MODE) void {
        self.mode = mode;
    }

    pub fn calculate(self: *Self, a: i32, b: i32) i32 {
        switch (self.mode) {
            ALU_MODE.ADD => {
                return a + b;
            },
            ALU_MODE.SUBTRACT => {
                return a - b;
            },
            ALU_MODE.AND => {
                return a & b;
            },
            ALU_MODE.OR => {
                return a | b;
            },
            ALU_MODE.SET_LESS_THAN => {
                return @intFromBool(a < b);
            },
        }
    }
};

test ALU {
    var alu = ALU{};
    alu.setMode(ALU.ALU_MODE.ADD);
    try expect(alu.calculate(1, 3) == 4);
    alu.setMode(ALU.ALU_MODE.SUBTRACT);
    try expect(alu.calculate(4, 1) == 3);
    alu.setMode(ALU.ALU_MODE.AND);
    try expect(alu.calculate(3, 2) == 2);
    alu.setMode(ALU.ALU_MODE.OR);
    try expect(alu.calculate(4, 5) == 5);
    alu.setMode(ALU.ALU_MODE.SET_LESS_THAN);
    try expect(alu.calculate(3, 9) == 1);
}

pub const Model = struct {
    const Self = @This();
    allocator: std.mem.Allocator,
    instruction_memory: *Memory,
    data_memory: *Memory,
    register_file: RegisterFile,
    control_unit: *ControlUnit,
    pc: u32,

    pub fn init(allocator: std.mem.Allocator, mem_size: u32, instruction_memory: *Memory) !*Self {
        var self = try allocator.create(Self);
        self.allocator = allocator;
        self.instruction_memory = instruction_memory;
        self.data_memory = try Memory.init(allocator, mem_size);
        self.register_file = try RegisterFile.init(allocator);
        self.control_unit = try ControlUnit.init(allocator);
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
        std.debug.print("Current Instruction 0x{1x:0>8}: 0b{0b:0>32}\n", .{ try self.instruction_memory.readWord(self.pc), self.pc });
        const op: u32 = instr & 0b1111111;
        const funct3: u32 = (instr >> 12) & 0b111;
        const funct7: u32 = (instr >> 25) & 0b1111111;
        const a1: u32 = (instr >> 15) & 0b11111;
        const a2: u32 = (instr >> 20) & 0b11111;
        const a3: u32 = (instr >> 7) & 0b11111;
        _ = funct3;
        _ = funct7;

        const rs1 = self.register_file.read(@truncate(a2));
        const rs2 = self.register_file.read(@truncate(a1));
        const rd = self.register_file.read(@truncate(a3));
        _ = rs1;
        _ = rs2;
        _ = rd;

        const control_signals: ControlSignals = try self.control_unit.decode(op);
        std.debug.print("Control Signals: {any}\n", .{control_signals});
        self.pc += 4;
    }
};

test Model {
    const allocator = std.testing.allocator;
    const instruction_memory = try Memory.init(allocator, 1024);
    const model = try Model.init(allocator, 1024, instruction_memory);
    defer model.deinit();

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
