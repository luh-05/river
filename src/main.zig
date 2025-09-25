const std = @import("std");

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

pub const Model = struct {
    const Self = @This();
    allocator: std.mem.Allocator,
    instruction_memory: *Memory,
    data_memory: *Memory,
    register_file: struct {
        registers: []i32,

        pub fn read(self: *Self, id: u8) i32 {
            if (id > 31) return 0;
            if (id == 0) return 0;
            return self.register_file.registers[id - 1];
        }
        pub fn write(self: *Self, id: u8, value: i32) void {
            if (id > 31) return;
            if (id == 0) return;
            self.register_file.registers[id - 1] = value;
        }
    },
    pc: u32,

    pub fn init(allocator: std.mem.Allocator, mem_size: u32, instruction_memory: *Memory) !*Self {
        var self = try allocator.create(Self);
        self.allocator = allocator;
        self.instruction_memory = instruction_memory;
        self.data_memory = try Memory.init(allocator, mem_size);
        self.register_file.registers = try allocator.alloc(i32, 31);
        self.pc = 0x0;
        return self;
    }

    pub fn deinit(self: *Self) void {
        self.instruction_memory.deinit();
        self.data_memory.deinit();
        self.allocator.free(self.register_file.registers);
        self.allocator.destroy(self);
    }

    pub fn tick(self: *Self) !void {
        const instr: [4]u8 = @bitCast(try self.instruction_memory.readWord(self.pc));
        std.debug.print("Current Instruction: 0b{0b:0>8}", .{try self.instruction_memory.readWord(self.pc)});
        const op: u8 = instr[3] & ~(1 << 7);
        const funct3: u8 = (instr[2] >> 4) & ~(11111 << 3);
        const funct7: u8 = instr[1] & ~(1);
        const a1_temp: u16 = @bitCast(instr[2..3]);
        const a1: u8 = @truncate((a1_temp >> 7) & ~(111 << 5));
        // const a2 = instr[20..24];
        // const a3 = instr[7..11];
        _ = op;
        _ = funct3;
        _ = funct7;
        _ = a1;
        // _ = a2;
        // _ = a3;
        self.pc += 4;
    }
};

test Model {
    const allocator = std.testing.allocator;
    const instruction_memory = try Memory.init(allocator, 1024);
    const model = try Model.init(allocator, 1024, instruction_memory);
    defer model.deinit();

    model.tick();
}

pub fn main() void {
    std.debug.print("Hello Zig!\n", .{});
    const allocator = std.heap.ArenaAllocator.init(std.heap.page_allocator);
    const mem = try Memory.init(allocator, 1024);
    mem.deinit();
}
