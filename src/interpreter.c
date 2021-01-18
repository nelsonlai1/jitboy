#include <inttypes.h>
#include <sys/mman.h>
#include <unistd.h>

#include "core.h"
#include "interrupt.h"

uint8_t __gb_read(gb_memory *mem, uint16_t addr)
{
    uint8_t ret;
    ret = mem->mem[addr];
    return ret;
}

static uint8_t __gb_execute_cb(gb_vm *vm)
{
    uint8_t inst_cycles;
    uint8_t cbop = __gb_read(&vm->memory, vm->state.pc++);
    uint8_t r = (cbop & 0x7);
    uint8_t b = (cbop >> 3) & 0x7;
    uint8_t d = (cbop >> 3) & 0x1;
    uint8_t val;
    uint8_t writeback = 1;

    inst_cycles = 8;
    /* Add an additional 8 cycles to these sets of instructions. */
    switch (cbop & 0xC7) {
    case 0x06:
    case 0x86:
    case 0xC6:
        inst_cycles += 8;
        break;
    case 0x46:
        inst_cycles += 4;
        break;
    }

    switch (r) {
    case 0:
        val = vm->state.b;
        break;

    case 1:
        val = vm->state.c;
        break;

    case 2:
        val = vm->state.d;
        break;

    case 3:
        val = vm->state.e;
        break;

    case 4:
        val = vm->state.h;
        break;

    case 5:
        val = vm->state.l;
        break;

    case 6:
        val = __gb_read(&vm->memory, vm->state.hl);
        break;

    /* Only values 0-7 are possible here, so we make the final case
     * default to satisfy -Wmaybe-uninitialized warning. */
    default:
        val = vm->state.a;
        break;
    }

    /* TODO: Find out WTF this is doing. */
    switch (cbop >> 6) {
    case 0x0:
        cbop = (cbop >> 4) & 0x3;

        switch (cbop) {
        case 0x0:    /* RdC R */
        case 0x1:    /* Rd R */
            if (d) { /* RRC R / RR R */
                uint8_t temp = val;
                val = (val >> 1);
                val |= cbop ? (vm->state.f_bits.c << 7) : (temp << 7);
                vm->state.f_bits.z = (val == 0x00);
                vm->state.f_bits.n = 0;
                vm->state.f_bits.h = 0;
                vm->state.f_bits.c = (temp & 0x01);
            } else { /* RLC R / RL R */
                uint8_t temp = val;
                val = (val << 1);
                val |= cbop ? vm->state.f_bits.c : (temp >> 7);
                vm->state.f_bits.z = (val == 0x00);
                vm->state.f_bits.n = 0;
                vm->state.f_bits.h = 0;
                vm->state.f_bits.c = (temp >> 7);
            }
            break;

        case 0x2:
            if (d) { /* SRA R */
                vm->state.f_bits.c = val & 0x01;
                val = (val >> 1) | (val & 0x80);
                vm->state.f_bits.z = (val == 0x00);
                vm->state.f_bits.n = 0;
                vm->state.f_bits.h = 0;
            } else { /* SLA R */
                vm->state.f_bits.c = (val >> 7);
                val = val << 1;
                vm->state.f_bits.z = (val == 0x00);
                vm->state.f_bits.n = 0;
                vm->state.f_bits.h = 0;
            }
            break;

        case 0x3:
            if (d) { /* SRL R */
                vm->state.f_bits.c = val & 0x01;
                val = val >> 1;
                vm->state.f_bits.z = (val == 0x00);
                vm->state.f_bits.n = 0;
                vm->state.f_bits.h = 0;
            } else { /* SWAP R */
                uint8_t temp = (val >> 4) & 0x0F;
                temp |= (val << 4) & 0xF0;
                val = temp;
                vm->state.f_bits.z = (val == 0x00);
                vm->state.f_bits.n = 0;
                vm->state.f_bits.h = 0;
                vm->state.f_bits.c = 0;
            }
            break;
        }
        break;

    case 0x1: /* BIT B, R */
        vm->state.f_bits.z = !((val >> b) & 0x1);
        vm->state.f_bits.n = 0;
        vm->state.f_bits.h = 1;
        writeback = 0;
        break;

    case 0x2: /* RES B, R */
        val &= (0xFE << b) | (0xFF >> (8 - b));
        break;

    case 0x3: /* SET B, R */
        val |= (0x1 << b);
        break;
    }

    if (writeback) {
        switch (r) {
        case 0:
            vm->state.b = val;
            break;

        case 1:
            vm->state.c = val;
            break;

        case 2:
            vm->state.d = val;
            break;

        case 3:
            vm->state.e = val;
            break;

        case 4:
            vm->state.h = val;
            break;

        case 5:
            vm->state.l = val;
            break;

        case 6:
            gb_memory_write(&vm->state, vm->state.hl, val);
            break;

        case 7:
            vm->state.a = val;
            break;
        }
    }
    return inst_cycles;
}

/* Internal function used to step the CPU */
void __gb_step_cpu(gb_vm *vm)
{
    uint8_t opcode, inst_cycles;
    static const uint8_t op_cycles[0x100] = {
        /* clang-format off */
        /*          0   1   2   3   4   5   6   7   8   9   A   B   C   D   E   F */
        /* 0x00 */  4, 12,  8,  8,  4,  4,  8,  4, 20,  8,  8,  8,  4,  4,  8,  4,
        /* 0x10 */  4, 12,  8,  8,  4,  4,  8,  4, 12,  8,  8,  8,  4,  4,  8,  4,
        /* 0x20 */  8, 12,  8,  8,  4,  4,  8,  4,  8,  8,  8,  8,  4,  4,  8,  4,
        /* 0x30 */  8, 12,  8,  8, 12, 12, 12,  4,  8,  8,  8,  8,  4,  4,  8,  4,
        /* 0x40 */  4,  4,  4,  4,  4,  4,  8,  4,  4,  4,  4,  4,  4,  4,  8,  4,
        /* 0x50 */  4,  4,  4,  4,  4,  4,  8,  4,  4,  4,  4,  4,  4,  4,  8,  4,
        /* 0x60 */  4,  4,  4,  4,  4,  4,  8,  4,  4,  4,  4,  4,  4,  4,  8,  4,
        /* 0x70 */  8,  8,  8,  8,  8,  8,  4,  8,  4,  4,  4,  4,  4,  4,  8,  4,
        /* 0x80 */  4,  4,  4,  4,  4,  4,  8,  4,  4,  4,  4,  4,  4,  4,  8,  4,
        /* 0x90 */  4,  4,  4,  4,  4,  4,  8,  4,  4,  4,  4,  4,  4,  4,  8,  4,
        /* 0xA0 */  4,  4,  4,  4,  4,  4,  8,  4,  4,  4,  4,  4,  4,  4,  8,  4,
        /* 0xB0 */  4,  4,  4,  4,  4,  4,  8,  4,  4,  4,  4,  4,  4,  4,  8,  4,
        /* 0xC0 */  8, 12, 12, 16, 12, 16,  8, 16,  8, 16, 12,  8, 12, 24,  8, 16,
        /* 0xD0 */  8, 12, 12,  0, 12, 16,  8, 16,  8, 16, 12,  0, 12,  0,  8, 16,
        /* 0xE0 */ 12, 12,  8,  0,  0, 16,  8, 16, 16,  4, 16,  0,  0,  0,  8, 16,
        /* 0xF0 */ 12, 12,  8,  4,  0, 16,  8, 16, 12,  8, 16,  4,  0,  0,  8, 16,
        /* clang-format on */
    };

    /* Handle interrupts */
    /*
    if ((vm->state.ime || vm->state.halt) &&
        (gb->gb_reg.IF & gb->gb_reg.IE & ANY_INTR)) {
        vm->state.halt = 0;

        if (vm->state.ime) {
            // Disable interrupts
            vm->state.ime = 0;

            // Push program counter
            gb_memory_write(&vm->state, --vm->state._sp, vm->state.pc >> 8);
            gb_memory_write(&vm->state, --vm->state._sp, vm->state.pc & 0xFF);

            // Call interrupt handler if required.
            if (gb->gb_reg.IF & gb->gb_reg.IE & VBLANK_INTR) {
                vm->state.pc = VBLANK_INTR_ADDR;
                gb->gb_reg.IF ^= VBLANK_INTR;
            } else if (gb->gb_reg.IF & gb->gb_reg.IE & LCDC_INTR) {
                vm->state.pc = LCDC_INTR_ADDR;
                gb->gb_reg.IF ^= LCDC_INTR;
            } else if (gb->gb_reg.IF & gb->gb_reg.IE & TIMER_INTR) {
                vm->state.pc = TIMER_INTR_ADDR;
                gb->gb_reg.IF ^= TIMER_INTR;
            } else if (gb->gb_reg.IF & gb->gb_reg.IE & SERIAL_INTR) {
                vm->state.pc = SERIAL_INTR_ADDR;
                gb->gb_reg.IF ^= SERIAL_INTR;
            } else if (gb->gb_reg.IF & gb->gb_reg.IE & CONTROL_INTR) {
                vm->state.pc = CONTROL_INTR_ADDR;
                gb->gb_reg.IF ^= CONTROL_INTR;
            }
        }
    }
    */

    /* Obtain opcode */
    opcode = (vm->state.halt ? 0x00 : __gb_read(&vm->memory, vm->state.pc++));
    inst_cycles = op_cycles[opcode];

    /* Execute opcode */
    switch (opcode) {
    case 0x00: /* NOP */
        break;

    case 0x01: /* LD BC, imm */
        vm->state.c = __gb_read(&vm->memory, vm->state.pc++);
        vm->state.b = __gb_read(&vm->memory, vm->state.pc++);
        break;

    case 0x02: /* LD (BC), A */
        gb_memory_write(&vm->state, vm->state.bc, vm->state.a);
        break;

    case 0x03: /* INC BC */
        vm->state.bc++;
        break;

    case 0x04: /* INC B */
        vm->state.b++;
        vm->state.f_bits.z = (vm->state.b == 0x00);
        vm->state.f_bits.n = 0;
        vm->state.f_bits.h = ((vm->state.b & 0x0F) == 0x00);
        break;

    case 0x05: /* DEC B */
        vm->state.b--;
        vm->state.f_bits.z = (vm->state.b == 0x00);
        vm->state.f_bits.n = 1;
        vm->state.f_bits.h = ((vm->state.b & 0x0F) == 0x0F);
        break;

    case 0x06: /* LD B, imm */
        vm->state.b = __gb_read(&vm->memory, vm->state.pc++);
        break;

    case 0x07: /* RLCA */
        vm->state.a = (vm->state.a << 1) | (vm->state.a >> 7);
        vm->state.f_bits.z = 0;
        vm->state.f_bits.n = 0;
        vm->state.f_bits.h = 0;
        vm->state.f_bits.c = (vm->state.a & 0x01);
        break;

    case 0x08: { /* LD (imm), SP */
        uint16_t temp = __gb_read(&vm->memory, vm->state.pc++);
        temp |= __gb_read(&vm->memory, vm->state.pc++) << 8;
        gb_memory_write(&vm->state, temp++, vm->state._sp & 0xFF);
        gb_memory_write(&vm->state, temp, vm->state._sp >> 8);
        break;
    }

    case 0x09: { /* ADD HL, BC */
        uint_fast32_t temp = vm->state.hl + vm->state.bc;
        vm->state.f_bits.n = 0;
        vm->state.f_bits.h =
            (temp ^ vm->state.hl ^ vm->state.bc) & 0x1000 ? 1 : 0;
        vm->state.f_bits.c = (temp & 0xFFFF0000) ? 1 : 0;
        vm->state.hl = (temp & 0x0000FFFF);
        break;
    }

    case 0x0A: /* LD A, (BC) */
        vm->state.a = __gb_read(&vm->memory, vm->state.bc);
        break;

    case 0x0B: /* DEC BC */
        vm->state.bc--;
        break;

    case 0x0C: /* INC C */
        vm->state.c++;
        vm->state.f_bits.z = (vm->state.c == 0x00);
        vm->state.f_bits.n = 0;
        vm->state.f_bits.h = ((vm->state.c & 0x0F) == 0x00);
        break;

    case 0x0D: /* DEC C */
        vm->state.c--;
        vm->state.f_bits.z = (vm->state.c == 0x00);
        vm->state.f_bits.n = 1;
        vm->state.f_bits.h = ((vm->state.c & 0x0F) == 0x0F);
        break;

    case 0x0E: /* LD C, imm */
        vm->state.c = __gb_read(&vm->memory, vm->state.pc++);
        break;

    case 0x0F: /* RRCA */
        vm->state.f_bits.c = vm->state.a & 0x01;
        vm->state.a = (vm->state.a >> 1) | (vm->state.a << 7);
        vm->state.f_bits.z = 0;
        vm->state.f_bits.n = 0;
        vm->state.f_bits.h = 0;
        break;

    case 0x10: /* STOP */
        // vm->state.halt = 1;
        break;

    case 0x11: /* LD DE, imm */
        vm->state.e = __gb_read(&vm->memory, vm->state.pc++);
        vm->state.d = __gb_read(&vm->memory, vm->state.pc++);
        break;

    case 0x12: /* LD (DE), A */
        gb_memory_write(&vm->state, vm->state.de, vm->state.a);
        break;

    case 0x13: /* INC DE */
        vm->state.de++;
        break;

    case 0x14: /* INC D */
        vm->state.d++;
        vm->state.f_bits.z = (vm->state.d == 0x00);
        vm->state.f_bits.n = 0;
        vm->state.f_bits.h = ((vm->state.d & 0x0F) == 0x00);
        break;

    case 0x15: /* DEC D */
        vm->state.d--;
        vm->state.f_bits.z = (vm->state.d == 0x00);
        vm->state.f_bits.n = 1;
        vm->state.f_bits.h = ((vm->state.d & 0x0F) == 0x0F);
        break;

    case 0x16: /* LD D, imm */
        vm->state.d = __gb_read(&vm->memory, vm->state.pc++);
        break;

    case 0x17: { /* RLA */
        uint8_t temp = vm->state.a;
        vm->state.a = (vm->state.a << 1) | vm->state.f_bits.c;
        vm->state.f_bits.z = 0;
        vm->state.f_bits.n = 0;
        vm->state.f_bits.h = 0;
        vm->state.f_bits.c = (temp >> 7) & 0x01;
        break;
    }

    case 0x18: { /* JR imm */
        int8_t temp = (int8_t) __gb_read(&vm->memory, vm->state.pc++);
        vm->state.pc += temp;
        break;
    }

    case 0x19: { /* ADD HL, DE */
        uint_fast32_t temp = vm->state.hl + vm->state.de;
        vm->state.f_bits.n = 0;
        vm->state.f_bits.h =
            (temp ^ vm->state.hl ^ vm->state.de) & 0x1000 ? 1 : 0;
        vm->state.f_bits.c = (temp & 0xFFFF0000) ? 1 : 0;
        vm->state.hl = (temp & 0x0000FFFF);
        break;
    }

    case 0x1A: /* LD A, (DE) */
        vm->state.a = __gb_read(&vm->memory, vm->state.de);
        break;

    case 0x1B: /* DEC DE */
        vm->state.de--;
        break;

    case 0x1C: /* INC E */
        vm->state.e++;
        vm->state.f_bits.z = (vm->state.e == 0x00);
        vm->state.f_bits.n = 0;
        vm->state.f_bits.h = ((vm->state.e & 0x0F) == 0x00);
        break;

    case 0x1D: /* DEC E */
        vm->state.e--;
        vm->state.f_bits.z = (vm->state.e == 0x00);
        vm->state.f_bits.n = 1;
        vm->state.f_bits.h = ((vm->state.e & 0x0F) == 0x0F);
        break;

    case 0x1E: /* LD E, imm */
        vm->state.e = __gb_read(&vm->memory, vm->state.pc++);
        break;

    case 0x1F: { /* RRA */
        uint8_t temp = vm->state.a;
        vm->state.a = vm->state.a >> 1 | (vm->state.f_bits.c << 7);
        vm->state.f_bits.z = 0;
        vm->state.f_bits.n = 0;
        vm->state.f_bits.h = 0;
        vm->state.f_bits.c = temp & 0x1;
        break;
    }

    case 0x20: /* JP NZ, imm */
        if (!vm->state.f_bits.z) {
            int8_t temp = (int8_t) __gb_read(&vm->memory, vm->state.pc++);
            vm->state.pc += temp;
            inst_cycles += 4;
        } else
            vm->state.pc++;

        break;

    case 0x21: /* LD HL, imm */
        vm->state.l = __gb_read(&vm->memory, vm->state.pc++);
        vm->state.h = __gb_read(&vm->memory, vm->state.pc++);
        break;

    case 0x22: /* LDI (HL), A */
        gb_memory_write(&vm->state, vm->state.hl, vm->state.a);
        vm->state.hl++;
        break;

    case 0x23: /* INC HL */
        vm->state.hl++;
        break;

    case 0x24: /* INC H */
        vm->state.h++;
        vm->state.f_bits.z = (vm->state.h == 0x00);
        vm->state.f_bits.n = 0;
        vm->state.f_bits.h = ((vm->state.h & 0x0F) == 0x00);
        break;

    case 0x25: /* DEC H */
        vm->state.h--;
        vm->state.f_bits.z = (vm->state.h == 0x00);
        vm->state.f_bits.n = 1;
        vm->state.f_bits.h = ((vm->state.h & 0x0F) == 0x0F);
        break;

    case 0x26: /* LD H, imm */
        vm->state.h = __gb_read(&vm->memory, vm->state.pc++);
        break;

    case 0x27: { /* DAA */
        uint16_t a = vm->state.a;

        if (vm->state.f_bits.n) {
            if (vm->state.f_bits.h)
                a = (a - 0x06) & 0xFF;

            if (vm->state.f_bits.c)
                a -= 0x60;
        } else {
            if (vm->state.f_bits.h || (a & 0x0F) > 9)
                a += 0x06;

            if (vm->state.f_bits.c || a > 0x9F)
                a += 0x60;
        }

        if ((a & 0x100) == 0x100)
            vm->state.f_bits.c = 1;

        vm->state.a = a;
        vm->state.f_bits.z = (vm->state.a == 0);
        vm->state.f_bits.h = 0;

        break;
    }

    case 0x28: /* JP Z, imm */
        if (vm->state.f_bits.z) {
            int8_t temp = (int8_t) __gb_read(&vm->memory, vm->state.pc++);
            vm->state.pc += temp;
            inst_cycles += 4;
        } else
            vm->state.pc++;

        break;

    case 0x29: { /* ADD HL, HL */
        uint_fast32_t temp = vm->state.hl + vm->state.hl;
        vm->state.f_bits.n = 0;
        vm->state.f_bits.h = (temp & 0x1000) ? 1 : 0;
        vm->state.f_bits.c = (temp & 0xFFFF0000) ? 1 : 0;
        vm->state.hl = (temp & 0x0000FFFF);
        break;
    }

    case 0x2A: /* LD A, (HL+) */
        vm->state.a = __gb_read(&vm->memory, vm->state.hl++);
        break;

    case 0x2B: /* DEC HL */
        vm->state.hl--;
        break;

    case 0x2C: /* INC L */
        vm->state.l++;
        vm->state.f_bits.z = (vm->state.l == 0x00);
        vm->state.f_bits.n = 0;
        vm->state.f_bits.h = ((vm->state.l & 0x0F) == 0x00);
        break;

    case 0x2D: /* DEC L */
        vm->state.l--;
        vm->state.f_bits.z = (vm->state.l == 0x00);
        vm->state.f_bits.n = 1;
        vm->state.f_bits.h = ((vm->state.l & 0x0F) == 0x0F);
        break;

    case 0x2E: /* LD L, imm */
        vm->state.l = __gb_read(&vm->memory, vm->state.pc++);
        break;

    case 0x2F: /* CPL */
        vm->state.a = ~vm->state.a;
        vm->state.f_bits.n = 1;
        vm->state.f_bits.h = 1;
        break;

    case 0x30: /* JP NC, imm */
        if (!vm->state.f_bits.c) {
            int8_t temp = (int8_t) __gb_read(&vm->memory, vm->state.pc++);
            vm->state.pc += temp;
            inst_cycles += 4;
        } else
            vm->state.pc++;

        break;

    case 0x31: /* LD SP, imm */
        vm->state._sp = __gb_read(&vm->memory, vm->state.pc++);
        vm->state._sp |= __gb_read(&vm->memory, vm->state.pc++) << 8;
        break;

    case 0x32: /* LD (HL), A */
        gb_memory_write(&vm->state, vm->state.hl, vm->state.a);
        vm->state.hl--;
        break;

    case 0x33: /* INC SP */
        vm->state._sp++;
        break;

    case 0x34: { /* INC (HL) */
        uint8_t temp = __gb_read(&vm->memory, vm->state.hl) + 1;
        vm->state.f_bits.z = (temp == 0x00);
        vm->state.f_bits.n = 0;
        vm->state.f_bits.h = ((temp & 0x0F) == 0x00);
        gb_memory_write(&vm->state, vm->state.hl, temp);
        break;
    }

    case 0x35: { /* DEC (HL) */
        uint8_t temp = __gb_read(&vm->memory, vm->state.hl) - 1;
        vm->state.f_bits.z = (temp == 0x00);
        vm->state.f_bits.n = 1;
        vm->state.f_bits.h = ((temp & 0x0F) == 0x0F);
        gb_memory_write(&vm->state, vm->state.hl, temp);
        break;
    }

    case 0x36: /* LD (HL), imm */
        gb_memory_write(&vm->state, vm->state.hl,
                        __gb_read(&vm->memory, vm->state.pc++));
        break;

    case 0x37: /* SCF */
        vm->state.f_bits.n = 0;
        vm->state.f_bits.h = 0;
        vm->state.f_bits.c = 1;
        break;

    case 0x38: /* JP C, imm */
        if (vm->state.f_bits.c) {
            int8_t temp = (int8_t) __gb_read(&vm->memory, vm->state.pc++);
            vm->state.pc += temp;
            inst_cycles += 4;
        } else
            vm->state.pc++;

        break;

    case 0x39: { /* ADD HL, SP */
        uint_fast32_t temp = vm->state.hl + vm->state._sp;
        vm->state.f_bits.n = 0;
        vm->state.f_bits.h =
            ((vm->state.hl & 0xFFF) + (vm->state._sp & 0xFFF)) & 0x1000 ? 1 : 0;
        vm->state.f_bits.c = temp & 0x10000 ? 1 : 0;
        vm->state.hl = (uint16_t) temp;
        break;
    }

    case 0x3A: /* LD A, (HL) */
        vm->state.a = __gb_read(&vm->memory, vm->state.hl--);
        break;

    case 0x3B: /* DEC SP */
        vm->state._sp--;
        break;

    case 0x3C: /* INC A */
        vm->state.a++;
        vm->state.f_bits.z = (vm->state.a == 0x00);
        vm->state.f_bits.n = 0;
        vm->state.f_bits.h = ((vm->state.a & 0x0F) == 0x00);
        break;

    case 0x3D: /* DEC A */
        vm->state.a--;
        vm->state.f_bits.z = (vm->state.a == 0x00);
        vm->state.f_bits.n = 1;
        vm->state.f_bits.h = ((vm->state.a & 0x0F) == 0x0F);
        break;

    case 0x3E: /* LD A, imm */
        vm->state.a = __gb_read(&vm->memory, vm->state.pc++);
        break;

    case 0x3F: /* CCF */
        vm->state.f_bits.n = 0;
        vm->state.f_bits.h = 0;
        vm->state.f_bits.c = ~vm->state.f_bits.c;
        break;

    case 0x40: /* LD B, B */
        break;

    case 0x41: /* LD B, C */
        vm->state.b = vm->state.c;
        break;

    case 0x42: /* LD B, D */
        vm->state.b = vm->state.d;
        break;

    case 0x43: /* LD B, E */
        vm->state.b = vm->state.e;
        break;

    case 0x44: /* LD B, H */
        vm->state.b = vm->state.h;
        break;

    case 0x45: /* LD B, L */
        vm->state.b = vm->state.l;
        break;

    case 0x46: /* LD B, (HL) */
        vm->state.b = __gb_read(&vm->memory, vm->state.hl);
        break;

    case 0x47: /* LD B, A */
        vm->state.b = vm->state.a;
        break;

    case 0x48: /* LD C, B */
        vm->state.c = vm->state.b;
        break;

    case 0x49: /* LD C, C */
        break;

    case 0x4A: /* LD C, D */
        vm->state.c = vm->state.d;
        break;

    case 0x4B: /* LD C, E */
        vm->state.c = vm->state.e;
        break;

    case 0x4C: /* LD C, H */
        vm->state.c = vm->state.h;
        break;

    case 0x4D: /* LD C, L */
        vm->state.c = vm->state.l;
        break;

    case 0x4E: /* LD C, (HL) */
        vm->state.c = __gb_read(&vm->memory, vm->state.hl);
        break;

    case 0x4F: /* LD C, A */
        vm->state.c = vm->state.a;
        break;

    case 0x50: /* LD D, B */
        vm->state.d = vm->state.b;
        break;

    case 0x51: /* LD D, C */
        vm->state.d = vm->state.c;
        break;

    case 0x52: /* LD D, D */
        break;

    case 0x53: /* LD D, E */
        vm->state.d = vm->state.e;
        break;

    case 0x54: /* LD D, H */
        vm->state.d = vm->state.h;
        break;

    case 0x55: /* LD D, L */
        vm->state.d = vm->state.l;
        break;

    case 0x56: /* LD D, (HL) */
        vm->state.d = __gb_read(&vm->memory, vm->state.hl);
        break;

    case 0x57: /* LD D, A */
        vm->state.d = vm->state.a;
        break;

    case 0x58: /* LD E, B */
        vm->state.e = vm->state.b;
        break;

    case 0x59: /* LD E, C */
        vm->state.e = vm->state.c;
        break;

    case 0x5A: /* LD E, D */
        vm->state.e = vm->state.d;
        break;

    case 0x5B: /* LD E, E */
        break;

    case 0x5C: /* LD E, H */
        vm->state.e = vm->state.h;
        break;

    case 0x5D: /* LD E, L */
        vm->state.e = vm->state.l;
        break;

    case 0x5E: /* LD E, (HL) */
        vm->state.e = __gb_read(&vm->memory, vm->state.hl);
        break;

    case 0x5F: /* LD E, A */
        vm->state.e = vm->state.a;
        break;

    case 0x60: /* LD H, B */
        vm->state.h = vm->state.b;
        break;

    case 0x61: /* LD H, C */
        vm->state.h = vm->state.c;
        break;

    case 0x62: /* LD H, D */
        vm->state.h = vm->state.d;
        break;

    case 0x63: /* LD H, E */
        vm->state.h = vm->state.e;
        break;

    case 0x64: /* LD H, H */
        break;

    case 0x65: /* LD H, L */
        vm->state.h = vm->state.l;
        break;

    case 0x66: /* LD H, (HL) */
        vm->state.h = __gb_read(&vm->memory, vm->state.hl);
        break;

    case 0x67: /* LD H, A */
        vm->state.h = vm->state.a;
        break;

    case 0x68: /* LD L, B */
        vm->state.l = vm->state.b;
        break;

    case 0x69: /* LD L, C */
        vm->state.l = vm->state.c;
        break;

    case 0x6A: /* LD L, D */
        vm->state.l = vm->state.d;
        break;

    case 0x6B: /* LD L, E */
        vm->state.l = vm->state.e;
        break;

    case 0x6C: /* LD L, H */
        vm->state.l = vm->state.h;
        break;

    case 0x6D: /* LD L, L */
        break;

    case 0x6E: /* LD L, (HL) */
        vm->state.l = __gb_read(&vm->memory, vm->state.hl);
        break;

    case 0x6F: /* LD L, A */
        vm->state.l = vm->state.a;
        break;

    case 0x70: /* LD (HL), B */
        gb_memory_write(&vm->state, vm->state.hl, vm->state.b);
        break;

    case 0x71: /* LD (HL), C */
        gb_memory_write(&vm->state, vm->state.hl, vm->state.c);
        break;

    case 0x72: /* LD (HL), D */
        gb_memory_write(&vm->state, vm->state.hl, vm->state.d);
        break;

    case 0x73: /* LD (HL), E */
        gb_memory_write(&vm->state, vm->state.hl, vm->state.e);
        break;

    case 0x74: /* LD (HL), H */
        gb_memory_write(&vm->state, vm->state.hl, vm->state.h);
        break;

    case 0x75: /* LD (HL), L */
        gb_memory_write(&vm->state, vm->state.hl, vm->state.l);
        break;

    case 0x76: /* HALT */
        /* TODO: Emulate HALT bug? */
        vm->state.halt = 1;
        break;

    case 0x77: /* LD (HL), A */
        gb_memory_write(&vm->state, vm->state.hl, vm->state.a);
        break;

    case 0x78: /* LD A, B */
        vm->state.a = vm->state.b;
        break;

    case 0x79: /* LD A, C */
        vm->state.a = vm->state.c;
        break;

    case 0x7A: /* LD A, D */
        vm->state.a = vm->state.d;
        break;

    case 0x7B: /* LD A, E */
        vm->state.a = vm->state.e;
        break;

    case 0x7C: /* LD A, H */
        vm->state.a = vm->state.h;
        break;

    case 0x7D: /* LD A, L */
        vm->state.a = vm->state.l;
        break;

    case 0x7E: /* LD A, (HL) */
        vm->state.a = __gb_read(&vm->memory, vm->state.hl);
        break;

    case 0x7F: /* LD A, A */
        break;

    case 0x80: { /* ADD A, B */
        uint16_t temp = vm->state.a + vm->state.b;
        vm->state.f_bits.z = ((temp & 0xFF) == 0x00);
        vm->state.f_bits.n = 0;
        vm->state.f_bits.h = (vm->state.a ^ vm->state.b ^ temp) & 0x10 ? 1 : 0;
        vm->state.f_bits.c = (temp & 0xFF00) ? 1 : 0;
        vm->state.a = (temp & 0xFF);
        break;
    }

    case 0x81: { /* ADD A, C */
        uint16_t temp = vm->state.a + vm->state.c;
        vm->state.f_bits.z = ((temp & 0xFF) == 0x00);
        vm->state.f_bits.n = 0;
        vm->state.f_bits.h = (vm->state.a ^ vm->state.c ^ temp) & 0x10 ? 1 : 0;
        vm->state.f_bits.c = (temp & 0xFF00) ? 1 : 0;
        vm->state.a = (temp & 0xFF);
        break;
    }

    case 0x82: { /* ADD A, D */
        uint16_t temp = vm->state.a + vm->state.d;
        vm->state.f_bits.z = ((temp & 0xFF) == 0x00);
        vm->state.f_bits.n = 0;
        vm->state.f_bits.h = (vm->state.a ^ vm->state.d ^ temp) & 0x10 ? 1 : 0;
        vm->state.f_bits.c = (temp & 0xFF00) ? 1 : 0;
        vm->state.a = (temp & 0xFF);
        break;
    }

    case 0x83: { /* ADD A, E */
        uint16_t temp = vm->state.a + vm->state.e;
        vm->state.f_bits.z = ((temp & 0xFF) == 0x00);
        vm->state.f_bits.n = 0;
        vm->state.f_bits.h = (vm->state.a ^ vm->state.e ^ temp) & 0x10 ? 1 : 0;
        vm->state.f_bits.c = (temp & 0xFF00) ? 1 : 0;
        vm->state.a = (temp & 0xFF);
        break;
    }

    case 0x84: { /* ADD A, H */
        uint16_t temp = vm->state.a + vm->state.h;
        vm->state.f_bits.z = ((temp & 0xFF) == 0x00);
        vm->state.f_bits.n = 0;
        vm->state.f_bits.h = (vm->state.a ^ vm->state.h ^ temp) & 0x10 ? 1 : 0;
        vm->state.f_bits.c = (temp & 0xFF00) ? 1 : 0;
        vm->state.a = (temp & 0xFF);
        break;
    }

    case 0x85: { /* ADD A, L */
        uint16_t temp = vm->state.a + vm->state.l;
        vm->state.f_bits.z = ((temp & 0xFF) == 0x00);
        vm->state.f_bits.n = 0;
        vm->state.f_bits.h = (vm->state.a ^ vm->state.l ^ temp) & 0x10 ? 1 : 0;
        vm->state.f_bits.c = (temp & 0xFF00) ? 1 : 0;
        vm->state.a = (temp & 0xFF);
        break;
    }

    case 0x86: { /* ADD A, (HL) */
        uint8_t hl = __gb_read(&vm->memory, vm->state.hl);
        uint16_t temp = vm->state.a + hl;
        vm->state.f_bits.z = ((temp & 0xFF) == 0x00);
        vm->state.f_bits.n = 0;
        vm->state.f_bits.h = (vm->state.a ^ hl ^ temp) & 0x10 ? 1 : 0;
        vm->state.f_bits.c = (temp & 0xFF00) ? 1 : 0;
        vm->state.a = (temp & 0xFF);
        break;
    }

    case 0x87: { /* ADD A, A */
        uint16_t temp = vm->state.a + vm->state.a;
        vm->state.f_bits.z = ((temp & 0xFF) == 0x00);
        vm->state.f_bits.n = 0;
        vm->state.f_bits.h = temp & 0x10 ? 1 : 0;
        vm->state.f_bits.c = (temp & 0xFF00) ? 1 : 0;
        vm->state.a = (temp & 0xFF);
        break;
    }

    case 0x88: { /* ADC A, B */
        uint16_t temp = vm->state.a + vm->state.b + vm->state.f_bits.c;
        vm->state.f_bits.z = ((temp & 0xFF) == 0x00);
        vm->state.f_bits.n = 0;
        vm->state.f_bits.h = (vm->state.a ^ vm->state.b ^ temp) & 0x10 ? 1 : 0;
        vm->state.f_bits.c = (temp & 0xFF00) ? 1 : 0;
        vm->state.a = (temp & 0xFF);
        break;
    }

    case 0x89: { /* ADC A, C */
        uint16_t temp = vm->state.a + vm->state.c + vm->state.f_bits.c;
        vm->state.f_bits.z = ((temp & 0xFF) == 0x00);
        vm->state.f_bits.n = 0;
        vm->state.f_bits.h = (vm->state.a ^ vm->state.c ^ temp) & 0x10 ? 1 : 0;
        vm->state.f_bits.c = (temp & 0xFF00) ? 1 : 0;
        vm->state.a = (temp & 0xFF);
        break;
    }

    case 0x8A: { /* ADC A, D */
        uint16_t temp = vm->state.a + vm->state.d + vm->state.f_bits.c;
        vm->state.f_bits.z = ((temp & 0xFF) == 0x00);
        vm->state.f_bits.n = 0;
        vm->state.f_bits.h = (vm->state.a ^ vm->state.d ^ temp) & 0x10 ? 1 : 0;
        vm->state.f_bits.c = (temp & 0xFF00) ? 1 : 0;
        vm->state.a = (temp & 0xFF);
        break;
    }

    case 0x8B: { /* ADC A, E */
        uint16_t temp = vm->state.a + vm->state.e + vm->state.f_bits.c;
        vm->state.f_bits.z = ((temp & 0xFF) == 0x00);
        vm->state.f_bits.n = 0;
        vm->state.f_bits.h = (vm->state.a ^ vm->state.e ^ temp) & 0x10 ? 1 : 0;
        vm->state.f_bits.c = (temp & 0xFF00) ? 1 : 0;
        vm->state.a = (temp & 0xFF);
        break;
    }

    case 0x8C: { /* ADC A, H */
        uint16_t temp = vm->state.a + vm->state.h + vm->state.f_bits.c;
        vm->state.f_bits.z = ((temp & 0xFF) == 0x00);
        vm->state.f_bits.n = 0;
        vm->state.f_bits.h = (vm->state.a ^ vm->state.h ^ temp) & 0x10 ? 1 : 0;
        vm->state.f_bits.c = (temp & 0xFF00) ? 1 : 0;
        vm->state.a = (temp & 0xFF);
        break;
    }

    case 0x8D: { /* ADC A, L */
        uint16_t temp = vm->state.a + vm->state.l + vm->state.f_bits.c;
        vm->state.f_bits.z = ((temp & 0xFF) == 0x00);
        vm->state.f_bits.n = 0;
        vm->state.f_bits.h = (vm->state.a ^ vm->state.l ^ temp) & 0x10 ? 1 : 0;
        vm->state.f_bits.c = (temp & 0xFF00) ? 1 : 0;
        vm->state.a = (temp & 0xFF);
        break;
    }

    case 0x8E: { /* ADC A, (HL) */
        uint8_t val = __gb_read(&vm->memory, vm->state.hl);
        uint16_t temp = vm->state.a + val + vm->state.f_bits.c;
        vm->state.f_bits.z = ((temp & 0xFF) == 0x00);
        vm->state.f_bits.n = 0;
        vm->state.f_bits.h = (vm->state.a ^ val ^ temp) & 0x10 ? 1 : 0;
        vm->state.f_bits.c = (temp & 0xFF00) ? 1 : 0;
        vm->state.a = (temp & 0xFF);
        break;
    }

    case 0x8F: { /* ADC A, A */
        uint16_t temp = vm->state.a + vm->state.a + vm->state.f_bits.c;
        vm->state.f_bits.z = ((temp & 0xFF) == 0x00);
        vm->state.f_bits.n = 0;
        /* TODO: Optimization here? */
        vm->state.f_bits.h = (vm->state.a ^ vm->state.a ^ temp) & 0x10 ? 1 : 0;
        vm->state.f_bits.c = (temp & 0xFF00) ? 1 : 0;
        vm->state.a = (temp & 0xFF);
        break;
    }

    case 0x90: { /* SUB B */
        uint16_t temp = vm->state.a - vm->state.b;
        vm->state.f_bits.z = ((temp & 0xFF) == 0x00);
        vm->state.f_bits.n = 1;
        vm->state.f_bits.h = (vm->state.a ^ vm->state.b ^ temp) & 0x10 ? 1 : 0;
        vm->state.f_bits.c = (temp & 0xFF00) ? 1 : 0;
        vm->state.a = (temp & 0xFF);
        break;
    }

    case 0x91: { /* SUB C */
        uint16_t temp = vm->state.a - vm->state.c;
        vm->state.f_bits.z = ((temp & 0xFF) == 0x00);
        vm->state.f_bits.n = 1;
        vm->state.f_bits.h = (vm->state.a ^ vm->state.c ^ temp) & 0x10 ? 1 : 0;
        vm->state.f_bits.c = (temp & 0xFF00) ? 1 : 0;
        vm->state.a = (temp & 0xFF);
        break;
    }

    case 0x92: { /* SUB D */
        uint16_t temp = vm->state.a - vm->state.d;
        vm->state.f_bits.z = ((temp & 0xFF) == 0x00);
        vm->state.f_bits.n = 1;
        vm->state.f_bits.h = (vm->state.a ^ vm->state.d ^ temp) & 0x10 ? 1 : 0;
        vm->state.f_bits.c = (temp & 0xFF00) ? 1 : 0;
        vm->state.a = (temp & 0xFF);
        break;
    }

    case 0x93: { /* SUB E */
        uint16_t temp = vm->state.a - vm->state.e;
        vm->state.f_bits.z = ((temp & 0xFF) == 0x00);
        vm->state.f_bits.n = 1;
        vm->state.f_bits.h = (vm->state.a ^ vm->state.e ^ temp) & 0x10 ? 1 : 0;
        vm->state.f_bits.c = (temp & 0xFF00) ? 1 : 0;
        vm->state.a = (temp & 0xFF);
        break;
    }

    case 0x94: { /* SUB H */
        uint16_t temp = vm->state.a - vm->state.h;
        vm->state.f_bits.z = ((temp & 0xFF) == 0x00);
        vm->state.f_bits.n = 1;
        vm->state.f_bits.h = (vm->state.a ^ vm->state.h ^ temp) & 0x10 ? 1 : 0;
        vm->state.f_bits.c = (temp & 0xFF00) ? 1 : 0;
        vm->state.a = (temp & 0xFF);
        break;
    }

    case 0x95: { /* SUB L */
        uint16_t temp = vm->state.a - vm->state.l;
        vm->state.f_bits.z = ((temp & 0xFF) == 0x00);
        vm->state.f_bits.n = 1;
        vm->state.f_bits.h = (vm->state.a ^ vm->state.l ^ temp) & 0x10 ? 1 : 0;
        vm->state.f_bits.c = (temp & 0xFF00) ? 1 : 0;
        vm->state.a = (temp & 0xFF);
        break;
    }

    case 0x96: { /* SUB (HL) */
        uint8_t val = __gb_read(&vm->memory, vm->state.hl);
        uint16_t temp = vm->state.a - val;
        vm->state.f_bits.z = ((temp & 0xFF) == 0x00);
        vm->state.f_bits.n = 1;
        vm->state.f_bits.h = (vm->state.a ^ val ^ temp) & 0x10 ? 1 : 0;
        vm->state.f_bits.c = (temp & 0xFF00) ? 1 : 0;
        vm->state.a = (temp & 0xFF);
        break;
    }

    case 0x97: /* SUB A */
        vm->state.a = 0;
        vm->state.f_bits.z = 1;
        vm->state.f_bits.n = 1;
        vm->state.f_bits.h = 0;
        vm->state.f_bits.c = 0;
        break;

    case 0x98: { /* SBC A, B */
        uint16_t temp = vm->state.a - vm->state.b - vm->state.f_bits.c;
        vm->state.f_bits.z = ((temp & 0xFF) == 0x00);
        vm->state.f_bits.n = 1;
        vm->state.f_bits.h = (vm->state.a ^ vm->state.b ^ temp) & 0x10 ? 1 : 0;
        vm->state.f_bits.c = (temp & 0xFF00) ? 1 : 0;
        vm->state.a = (temp & 0xFF);
        break;
    }

    case 0x99: { /* SBC A, C */
        uint16_t temp = vm->state.a - vm->state.c - vm->state.f_bits.c;
        vm->state.f_bits.z = ((temp & 0xFF) == 0x00);
        vm->state.f_bits.n = 1;
        vm->state.f_bits.h = (vm->state.a ^ vm->state.c ^ temp) & 0x10 ? 1 : 0;
        vm->state.f_bits.c = (temp & 0xFF00) ? 1 : 0;
        vm->state.a = (temp & 0xFF);
        break;
    }

    case 0x9A: { /* SBC A, D */
        uint16_t temp = vm->state.a - vm->state.d - vm->state.f_bits.c;
        vm->state.f_bits.z = ((temp & 0xFF) == 0x00);
        vm->state.f_bits.n = 1;
        vm->state.f_bits.h = (vm->state.a ^ vm->state.d ^ temp) & 0x10 ? 1 : 0;
        vm->state.f_bits.c = (temp & 0xFF00) ? 1 : 0;
        vm->state.a = (temp & 0xFF);
        break;
    }

    case 0x9B: { /* SBC A, E */
        uint16_t temp = vm->state.a - vm->state.e - vm->state.f_bits.c;
        vm->state.f_bits.z = ((temp & 0xFF) == 0x00);
        vm->state.f_bits.n = 1;
        vm->state.f_bits.h = (vm->state.a ^ vm->state.e ^ temp) & 0x10 ? 1 : 0;
        vm->state.f_bits.c = (temp & 0xFF00) ? 1 : 0;
        vm->state.a = (temp & 0xFF);
        break;
    }

    case 0x9C: { /* SBC A, H */
        uint16_t temp = vm->state.a - vm->state.h - vm->state.f_bits.c;
        vm->state.f_bits.z = ((temp & 0xFF) == 0x00);
        vm->state.f_bits.n = 1;
        vm->state.f_bits.h = (vm->state.a ^ vm->state.h ^ temp) & 0x10 ? 1 : 0;
        vm->state.f_bits.c = (temp & 0xFF00) ? 1 : 0;
        vm->state.a = (temp & 0xFF);
        break;
    }

    case 0x9D: { /* SBC A, L */
        uint16_t temp = vm->state.a - vm->state.l - vm->state.f_bits.c;
        vm->state.f_bits.z = ((temp & 0xFF) == 0x00);
        vm->state.f_bits.n = 1;
        vm->state.f_bits.h = (vm->state.a ^ vm->state.l ^ temp) & 0x10 ? 1 : 0;
        vm->state.f_bits.c = (temp & 0xFF00) ? 1 : 0;
        vm->state.a = (temp & 0xFF);
        break;
    }

    case 0x9E: { /* SBC A, (HL) */
        uint8_t val = __gb_read(&vm->memory, vm->state.hl);
        uint16_t temp = vm->state.a - val - vm->state.f_bits.c;
        vm->state.f_bits.z = ((temp & 0xFF) == 0x00);
        vm->state.f_bits.n = 1;
        vm->state.f_bits.h = (vm->state.a ^ val ^ temp) & 0x10 ? 1 : 0;
        vm->state.f_bits.c = (temp & 0xFF00) ? 1 : 0;
        vm->state.a = (temp & 0xFF);
        break;
    }

    case 0x9F: /* SBC A, A */
        vm->state.a = vm->state.f_bits.c ? 0xFF : 0x00;
        vm->state.f_bits.z = vm->state.f_bits.c ? 0x00 : 0x01;
        vm->state.f_bits.n = 1;
        vm->state.f_bits.h = vm->state.f_bits.c;
        break;

    case 0xA0: /* AND B */
        vm->state.a = vm->state.a & vm->state.b;
        vm->state.f_bits.z = (vm->state.a == 0x00);
        vm->state.f_bits.n = 0;
        vm->state.f_bits.h = 1;
        vm->state.f_bits.c = 0;
        break;

    case 0xA1: /* AND C */
        vm->state.a = vm->state.a & vm->state.c;
        vm->state.f_bits.z = (vm->state.a == 0x00);
        vm->state.f_bits.n = 0;
        vm->state.f_bits.h = 1;
        vm->state.f_bits.c = 0;
        break;

    case 0xA2: /* AND D */
        vm->state.a = vm->state.a & vm->state.d;
        vm->state.f_bits.z = (vm->state.a == 0x00);
        vm->state.f_bits.n = 0;
        vm->state.f_bits.h = 1;
        vm->state.f_bits.c = 0;
        break;

    case 0xA3: /* AND E */
        vm->state.a = vm->state.a & vm->state.e;
        vm->state.f_bits.z = (vm->state.a == 0x00);
        vm->state.f_bits.n = 0;
        vm->state.f_bits.h = 1;
        vm->state.f_bits.c = 0;
        break;

    case 0xA4: /* AND H */
        vm->state.a = vm->state.a & vm->state.h;
        vm->state.f_bits.z = (vm->state.a == 0x00);
        vm->state.f_bits.n = 0;
        vm->state.f_bits.h = 1;
        vm->state.f_bits.c = 0;
        break;

    case 0xA5: /* AND L */
        vm->state.a = vm->state.a & vm->state.l;
        vm->state.f_bits.z = (vm->state.a == 0x00);
        vm->state.f_bits.n = 0;
        vm->state.f_bits.h = 1;
        vm->state.f_bits.c = 0;
        break;

    case 0xA6: /* AND B */
        vm->state.a = vm->state.a & __gb_read(&vm->memory, vm->state.hl);
        vm->state.f_bits.z = (vm->state.a == 0x00);
        vm->state.f_bits.n = 0;
        vm->state.f_bits.h = 1;
        vm->state.f_bits.c = 0;
        break;

    case 0xA7: /* AND A */
        vm->state.f_bits.z = (vm->state.a == 0x00);
        vm->state.f_bits.n = 0;
        vm->state.f_bits.h = 1;
        vm->state.f_bits.c = 0;
        break;

    case 0xA8: /* XOR B */
        vm->state.a = vm->state.a ^ vm->state.b;
        vm->state.f_bits.z = (vm->state.a == 0x00);
        vm->state.f_bits.n = 0;
        vm->state.f_bits.h = 0;
        vm->state.f_bits.c = 0;
        break;

    case 0xA9: /* XOR C */
        vm->state.a = vm->state.a ^ vm->state.c;
        vm->state.f_bits.z = (vm->state.a == 0x00);
        vm->state.f_bits.n = 0;
        vm->state.f_bits.h = 0;
        vm->state.f_bits.c = 0;
        break;

    case 0xAA: /* XOR D */
        vm->state.a = vm->state.a ^ vm->state.d;
        vm->state.f_bits.z = (vm->state.a == 0x00);
        vm->state.f_bits.n = 0;
        vm->state.f_bits.h = 0;
        vm->state.f_bits.c = 0;
        break;

    case 0xAB: /* XOR E */
        vm->state.a = vm->state.a ^ vm->state.e;
        vm->state.f_bits.z = (vm->state.a == 0x00);
        vm->state.f_bits.n = 0;
        vm->state.f_bits.h = 0;
        vm->state.f_bits.c = 0;
        break;

    case 0xAC: /* XOR H */
        vm->state.a = vm->state.a ^ vm->state.h;
        vm->state.f_bits.z = (vm->state.a == 0x00);
        vm->state.f_bits.n = 0;
        vm->state.f_bits.h = 0;
        vm->state.f_bits.c = 0;
        break;

    case 0xAD: /* XOR L */
        vm->state.a = vm->state.a ^ vm->state.l;
        vm->state.f_bits.z = (vm->state.a == 0x00);
        vm->state.f_bits.n = 0;
        vm->state.f_bits.h = 0;
        vm->state.f_bits.c = 0;
        break;

    case 0xAE: /* XOR (HL) */
        vm->state.a = vm->state.a ^ __gb_read(&vm->memory, vm->state.hl);
        vm->state.f_bits.z = (vm->state.a == 0x00);
        vm->state.f_bits.n = 0;
        vm->state.f_bits.h = 0;
        vm->state.f_bits.c = 0;
        break;

    case 0xAF: /* XOR A */
        vm->state.a = 0x00;
        vm->state.f_bits.z = 1;
        vm->state.f_bits.n = 0;
        vm->state.f_bits.h = 0;
        vm->state.f_bits.c = 0;
        break;

    case 0xB0: /* OR B */
        vm->state.a = vm->state.a | vm->state.b;
        vm->state.f_bits.z = (vm->state.a == 0x00);
        vm->state.f_bits.n = 0;
        vm->state.f_bits.h = 0;
        vm->state.f_bits.c = 0;
        break;

    case 0xB1: /* OR C */
        vm->state.a = vm->state.a | vm->state.c;
        vm->state.f_bits.z = (vm->state.a == 0x00);
        vm->state.f_bits.n = 0;
        vm->state.f_bits.h = 0;
        vm->state.f_bits.c = 0;
        break;

    case 0xB2: /* OR D */
        vm->state.a = vm->state.a | vm->state.d;
        vm->state.f_bits.z = (vm->state.a == 0x00);
        vm->state.f_bits.n = 0;
        vm->state.f_bits.h = 0;
        vm->state.f_bits.c = 0;
        break;

    case 0xB3: /* OR E */
        vm->state.a = vm->state.a | vm->state.e;
        vm->state.f_bits.z = (vm->state.a == 0x00);
        vm->state.f_bits.n = 0;
        vm->state.f_bits.h = 0;
        vm->state.f_bits.c = 0;
        break;

    case 0xB4: /* OR H */
        vm->state.a = vm->state.a | vm->state.h;
        vm->state.f_bits.z = (vm->state.a == 0x00);
        vm->state.f_bits.n = 0;
        vm->state.f_bits.h = 0;
        vm->state.f_bits.c = 0;
        break;

    case 0xB5: /* OR L */
        vm->state.a = vm->state.a | vm->state.l;
        vm->state.f_bits.z = (vm->state.a == 0x00);
        vm->state.f_bits.n = 0;
        vm->state.f_bits.h = 0;
        vm->state.f_bits.c = 0;
        break;

    case 0xB6: /* OR (HL) */
        vm->state.a = vm->state.a | __gb_read(&vm->memory, vm->state.hl);
        vm->state.f_bits.z = (vm->state.a == 0x00);
        vm->state.f_bits.n = 0;
        vm->state.f_bits.h = 0;
        vm->state.f_bits.c = 0;
        break;

    case 0xB7: /* OR A */
        vm->state.f_bits.z = (vm->state.a == 0x00);
        vm->state.f_bits.n = 0;
        vm->state.f_bits.h = 0;
        vm->state.f_bits.c = 0;
        break;

    case 0xB8: { /* CP B */
        uint16_t temp = vm->state.a - vm->state.b;
        vm->state.f_bits.z = ((temp & 0xFF) == 0x00);
        vm->state.f_bits.n = 1;
        vm->state.f_bits.h = (vm->state.a ^ vm->state.b ^ temp) & 0x10 ? 1 : 0;
        vm->state.f_bits.c = (temp & 0xFF00) ? 1 : 0;
        break;
    }

    case 0xB9: { /* CP C */
        uint16_t temp = vm->state.a - vm->state.c;
        vm->state.f_bits.z = ((temp & 0xFF) == 0x00);
        vm->state.f_bits.n = 1;
        vm->state.f_bits.h = (vm->state.a ^ vm->state.c ^ temp) & 0x10 ? 1 : 0;
        vm->state.f_bits.c = (temp & 0xFF00) ? 1 : 0;
        break;
    }

    case 0xBA: { /* CP D */
        uint16_t temp = vm->state.a - vm->state.d;
        vm->state.f_bits.z = ((temp & 0xFF) == 0x00);
        vm->state.f_bits.n = 1;
        vm->state.f_bits.h = (vm->state.a ^ vm->state.d ^ temp) & 0x10 ? 1 : 0;
        vm->state.f_bits.c = (temp & 0xFF00) ? 1 : 0;
        break;
    }

    case 0xBB: { /* CP E */
        uint16_t temp = vm->state.a - vm->state.e;
        vm->state.f_bits.z = ((temp & 0xFF) == 0x00);
        vm->state.f_bits.n = 1;
        vm->state.f_bits.h = (vm->state.a ^ vm->state.e ^ temp) & 0x10 ? 1 : 0;
        vm->state.f_bits.c = (temp & 0xFF00) ? 1 : 0;
        break;
    }

    case 0xBC: { /* CP H */
        uint16_t temp = vm->state.a - vm->state.h;
        vm->state.f_bits.z = ((temp & 0xFF) == 0x00);
        vm->state.f_bits.n = 1;
        vm->state.f_bits.h = (vm->state.a ^ vm->state.h ^ temp) & 0x10 ? 1 : 0;
        vm->state.f_bits.c = (temp & 0xFF00) ? 1 : 0;
        break;
    }

    case 0xBD: { /* CP L */
        uint16_t temp = vm->state.a - vm->state.l;
        vm->state.f_bits.z = ((temp & 0xFF) == 0x00);
        vm->state.f_bits.n = 1;
        vm->state.f_bits.h = (vm->state.a ^ vm->state.l ^ temp) & 0x10 ? 1 : 0;
        vm->state.f_bits.c = (temp & 0xFF00) ? 1 : 0;
        break;
    }

    /* TODO: Optimization by combining similar opcode routines. */
    case 0xBE: { /* CP B */
        uint8_t val = __gb_read(&vm->memory, vm->state.hl);
        uint16_t temp = vm->state.a - val;
        vm->state.f_bits.z = ((temp & 0xFF) == 0x00);
        vm->state.f_bits.n = 1;
        vm->state.f_bits.h = (vm->state.a ^ val ^ temp) & 0x10 ? 1 : 0;
        vm->state.f_bits.c = (temp & 0xFF00) ? 1 : 0;
        break;
    }

    case 0xBF: /* CP A */
        vm->state.f_bits.z = 1;
        vm->state.f_bits.n = 1;
        vm->state.f_bits.h = 0;
        vm->state.f_bits.c = 0;
        break;

    case 0xC0: /* RET NZ */
        if (!vm->state.f_bits.z) {
            vm->state.pc = __gb_read(&vm->memory, vm->state._sp++);
            vm->state.pc |= __gb_read(&vm->memory, vm->state._sp++) << 8;
            inst_cycles += 12;
        }

        break;

    case 0xC1: /* POP BC */
        vm->state.c = __gb_read(&vm->memory, vm->state._sp++);
        vm->state.b = __gb_read(&vm->memory, vm->state._sp++);
        break;

    case 0xC2: /* JP NZ, imm */
        if (!vm->state.f_bits.z) {
            uint16_t temp = __gb_read(&vm->memory, vm->state.pc++);
            temp |= __gb_read(&vm->memory, vm->state.pc++) << 8;
            vm->state.pc = temp;
            inst_cycles += 4;
        } else
            vm->state.pc += 2;

        break;

    case 0xC3: { /* JP imm */
        uint16_t temp = __gb_read(&vm->memory, vm->state.pc++);
        temp |= __gb_read(&vm->memory, vm->state.pc) << 8;
        vm->state.pc = temp;
        break;
    }

    case 0xC4: /* CALL NZ imm */
        if (!vm->state.f_bits.z) {
            uint16_t temp = __gb_read(&vm->memory, vm->state.pc++);
            temp |= __gb_read(&vm->memory, vm->state.pc++) << 8;
            gb_memory_write(&vm->state, --vm->state._sp, vm->state.pc >> 8);
            gb_memory_write(&vm->state, --vm->state._sp, vm->state.pc & 0xFF);
            vm->state.pc = temp;
            inst_cycles += 12;
        } else
            vm->state.pc += 2;

        break;

    case 0xC5: /* PUSH BC */
        gb_memory_write(&vm->state, --vm->state._sp, vm->state.b);
        gb_memory_write(&vm->state, --vm->state._sp, vm->state.c);
        break;

    case 0xC6: { /* ADD A, imm */
        uint8_t value = __gb_read(&vm->memory, vm->state.pc++);
        uint16_t calc = vm->state.a + value;
        vm->state.f_bits.z = ((uint8_t) calc == 0) ? 1 : 0;
        vm->state.f_bits.h =
            ((vm->state.a & 0xF) + (value & 0xF) > 0x0F) ? 1 : 0;
        vm->state.f_bits.c = calc > 0xFF ? 1 : 0;
        vm->state.f_bits.n = 0;
        vm->state.a = (uint8_t) calc;
        break;
    }

    case 0xC7: /* RST 0x0000 */
        gb_memory_write(&vm->state, --vm->state._sp, vm->state.pc >> 8);
        gb_memory_write(&vm->state, --vm->state._sp, vm->state.pc & 0xFF);
        vm->state.pc = 0x0000;
        break;

    case 0xC8: /* RET Z */
        if (vm->state.f_bits.z) {
            uint16_t temp = __gb_read(&vm->memory, vm->state._sp++);
            temp |= __gb_read(&vm->memory, vm->state._sp++) << 8;
            vm->state.pc = temp;
            inst_cycles += 12;
        }

        break;

    case 0xC9: { /* RET */
        uint16_t temp = __gb_read(&vm->memory, vm->state._sp++);
        temp |= __gb_read(&vm->memory, vm->state._sp++) << 8;
        vm->state.pc = temp;
        break;
    }

    case 0xCA: /* JP Z, imm */
        if (vm->state.f_bits.z) {
            uint16_t temp = __gb_read(&vm->memory, vm->state.pc++);
            temp |= __gb_read(&vm->memory, vm->state.pc++) << 8;
            vm->state.pc = temp;
            inst_cycles += 4;
        } else
            vm->state.pc += 2;

        break;

    case 0xCB: /* CB INST */
        inst_cycles = __gb_execute_cb(vm);
        break;

    case 0xCC: /* CALL Z, imm */
        if (vm->state.f_bits.z) {
            uint16_t temp = __gb_read(&vm->memory, vm->state.pc++);
            temp |= __gb_read(&vm->memory, vm->state.pc++) << 8;
            gb_memory_write(&vm->state, --vm->state._sp, vm->state.pc >> 8);
            gb_memory_write(&vm->state, --vm->state._sp, vm->state.pc & 0xFF);
            vm->state.pc = temp;
            inst_cycles += 12;
        } else
            vm->state.pc += 2;

        break;

    case 0xCD: { /* CALL imm */
        uint16_t addr = __gb_read(&vm->memory, vm->state.pc++);
        addr |= __gb_read(&vm->memory, vm->state.pc++) << 8;
        gb_memory_write(&vm->state, --vm->state._sp, vm->state.pc >> 8);
        gb_memory_write(&vm->state, --vm->state._sp, vm->state.pc & 0xFF);
        vm->state.pc = addr;
    } break;

    case 0xCE: { /* ADC A, imm */
        uint8_t value, a, carry;
        value = __gb_read(&vm->memory, vm->state.pc++);
        a = vm->state.a;
        carry = vm->state.f_bits.c;
        vm->state.a = a + value + carry;

        vm->state.f_bits.z = vm->state.a == 0 ? 1 : 0;
        vm->state.f_bits.h = ((a & 0xF) + (value & 0xF) + carry > 0x0F) ? 1 : 0;
        vm->state.f_bits.c =
            (((uint16_t) a) + ((uint16_t) value) + carry > 0xFF) ? 1 : 0;
        vm->state.f_bits.n = 0;
        break;
    }

    case 0xCF: /* RST 0x0008 */
        gb_memory_write(&vm->state, --vm->state._sp, vm->state.pc >> 8);
        gb_memory_write(&vm->state, --vm->state._sp, vm->state.pc & 0xFF);
        vm->state.pc = 0x0008;
        break;

    case 0xD0: /* RET NC */
        if (!vm->state.f_bits.c) {
            uint16_t temp = __gb_read(&vm->memory, vm->state._sp++);
            temp |= __gb_read(&vm->memory, vm->state._sp++) << 8;
            vm->state.pc = temp;
            inst_cycles += 12;
        }

        break;

    case 0xD1: /* POP DE */
        vm->state.e = __gb_read(&vm->memory, vm->state._sp++);
        vm->state.d = __gb_read(&vm->memory, vm->state._sp++);
        break;

    case 0xD2: /* JP NC, imm */
        if (!vm->state.f_bits.c) {
            uint16_t temp = __gb_read(&vm->memory, vm->state.pc++);
            temp |= __gb_read(&vm->memory, vm->state.pc++) << 8;
            vm->state.pc = temp;
            inst_cycles += 4;
        } else
            vm->state.pc += 2;

        break;

    case 0xD4: /* CALL NC, imm */
        if (!vm->state.f_bits.c) {
            uint16_t temp = __gb_read(&vm->memory, vm->state.pc++);
            temp |= __gb_read(&vm->memory, vm->state.pc++) << 8;
            gb_memory_write(&vm->state, --vm->state._sp, vm->state.pc >> 8);
            gb_memory_write(&vm->state, --vm->state._sp, vm->state.pc & 0xFF);
            vm->state.pc = temp;
            inst_cycles += 12;
        } else
            vm->state.pc += 2;

        break;

    case 0xD5: /* PUSH DE */
        gb_memory_write(&vm->state, --vm->state._sp, vm->state.d);
        gb_memory_write(&vm->state, --vm->state._sp, vm->state.e);
        break;

    case 0xD6: { /* SUB imm */
        uint8_t val = __gb_read(&vm->memory, vm->state.pc++);
        uint16_t temp = vm->state.a - val;
        vm->state.f_bits.z = ((temp & 0xFF) == 0x00);
        vm->state.f_bits.n = 1;
        vm->state.f_bits.h = (vm->state.a ^ val ^ temp) & 0x10 ? 1 : 0;
        vm->state.f_bits.c = (temp & 0xFF00) ? 1 : 0;
        vm->state.a = (temp & 0xFF);
        break;
    }

    case 0xD7: /* RST 0x0010 */
        gb_memory_write(&vm->state, --vm->state._sp, vm->state.pc >> 8);
        gb_memory_write(&vm->state, --vm->state._sp, vm->state.pc & 0xFF);
        vm->state.pc = 0x0010;
        break;

    case 0xD8: /* RET C */
        if (vm->state.f_bits.c) {
            uint16_t temp = __gb_read(&vm->memory, vm->state._sp++);
            temp |= __gb_read(&vm->memory, vm->state._sp++) << 8;
            vm->state.pc = temp;
            inst_cycles += 12;
        }

        break;

    case 0xD9: { /* RETI */
        uint16_t temp = __gb_read(&vm->memory, vm->state._sp++);
        temp |= __gb_read(&vm->memory, vm->state._sp++) << 8;
        vm->state.pc = temp;
        vm->state.ime = 1;
    } break;

    case 0xDA: /* JP C, imm */
        if (vm->state.f_bits.c) {
            uint16_t addr = __gb_read(&vm->memory, vm->state.pc++);
            addr |= __gb_read(&vm->memory, vm->state.pc++) << 8;
            vm->state.pc = addr;
            inst_cycles += 4;
        } else
            vm->state.pc += 2;

        break;

    case 0xDC: /* CALL C, imm */
        if (vm->state.f_bits.c) {
            uint16_t temp = __gb_read(&vm->memory, vm->state.pc++);
            temp |= __gb_read(&vm->memory, vm->state.pc++) << 8;
            gb_memory_write(&vm->state, --vm->state._sp, vm->state.pc >> 8);
            gb_memory_write(&vm->state, --vm->state._sp, vm->state.pc & 0xFF);
            vm->state.pc = temp;
            inst_cycles += 12;
        } else
            vm->state.pc += 2;

        break;

    case 0xDE: { /* SBC A, imm */
        uint8_t temp_8 = __gb_read(&vm->memory, vm->state.pc++);
        uint16_t temp_16 = vm->state.a - temp_8 - vm->state.f_bits.c;
        vm->state.f_bits.z = ((temp_16 & 0xFF) == 0x00);
        vm->state.f_bits.n = 1;
        vm->state.f_bits.h = (vm->state.a ^ temp_8 ^ temp_16) & 0x10 ? 1 : 0;
        vm->state.f_bits.c = (temp_16 & 0xFF00) ? 1 : 0;
        vm->state.a = (temp_16 & 0xFF);
        break;
    }

    case 0xDF: /* RST 0x0018 */
        gb_memory_write(&vm->state, --vm->state._sp, vm->state.pc >> 8);
        gb_memory_write(&vm->state, --vm->state._sp, vm->state.pc & 0xFF);
        vm->state.pc = 0x0018;
        break;

    case 0xE0: /* LD (0xFF00+imm), A */
        gb_memory_write(&vm->state,
                        0xFF00 | __gb_read(&vm->memory, vm->state.pc++),
                        vm->state.a);
        break;

    case 0xE1: /* POP HL */
        vm->state.l = __gb_read(&vm->memory, vm->state._sp++);
        vm->state.h = __gb_read(&vm->memory, vm->state._sp++);
        break;

    case 0xE2: /* LD (C), A */
        gb_memory_write(&vm->state, 0xFF00 | vm->state.c, vm->state.a);
        break;

    case 0xE5: /* PUSH HL */
        gb_memory_write(&vm->state, --vm->state._sp, vm->state.h);
        gb_memory_write(&vm->state, --vm->state._sp, vm->state.l);
        break;

    case 0xE6: /* AND imm */
        /* TODO: Optimization? */
        vm->state.a = vm->state.a & __gb_read(&vm->memory, vm->state.pc++);
        vm->state.f_bits.z = (vm->state.a == 0x00);
        vm->state.f_bits.n = 0;
        vm->state.f_bits.h = 1;
        vm->state.f_bits.c = 0;
        break;

    case 0xE7: /* RST 0x0020 */
        gb_memory_write(&vm->state, --vm->state._sp, vm->state.pc >> 8);
        gb_memory_write(&vm->state, --vm->state._sp, vm->state.pc & 0xFF);
        vm->state.pc = 0x0020;
        break;

    case 0xE8: { /* ADD SP, imm */
        int8_t offset = (int8_t) __gb_read(&vm->memory, vm->state.pc++);
        /* TODO: Move flag assignments for optimization */
        vm->state.f_bits.z = 0;
        vm->state.f_bits.n = 0;
        vm->state.f_bits.h =
            ((vm->state._sp & 0xF) + (offset & 0xF) > 0xF) ? 1 : 0;
        vm->state.f_bits.c = ((vm->state._sp & 0xFF) + (offset & 0xFF) > 0xFF);
        vm->state._sp += offset;
        break;
    }

    case 0xE9: /* JP (HL) */
        vm->state.pc = vm->state.hl;
        break;

    case 0xEA: { /* LD (imm), A */
        uint16_t addr = __gb_read(&vm->memory, vm->state.pc++);
        addr |= __gb_read(&vm->memory, vm->state.pc++) << 8;
        gb_memory_write(&vm->state, addr, vm->state.a);
        break;
    }

    case 0xEE: /* XOR imm */
        vm->state.a = vm->state.a ^ __gb_read(&vm->memory, vm->state.pc++);
        vm->state.f_bits.z = (vm->state.a == 0x00);
        vm->state.f_bits.n = 0;
        vm->state.f_bits.h = 0;
        vm->state.f_bits.c = 0;
        break;

    case 0xEF: /* RST 0x0028 */
        gb_memory_write(&vm->state, --vm->state._sp, vm->state.pc >> 8);
        gb_memory_write(&vm->state, --vm->state._sp, vm->state.pc & 0xFF);
        vm->state.pc = 0x0028;
        break;

    case 0xF0: /* LD A, (0xFF00+imm) */
        vm->state.a = __gb_read(
            &vm->memory, 0xFF00 | __gb_read(&vm->memory, vm->state.pc++));
        break;

    case 0xF1: { /* POP AF */
        uint8_t temp_8 = __gb_read(&vm->memory, vm->state._sp++);
        vm->state.f_bits.z = (temp_8 >> 7) & 1;
        vm->state.f_bits.n = (temp_8 >> 6) & 1;
        vm->state.f_bits.h = (temp_8 >> 5) & 1;
        vm->state.f_bits.c = (temp_8 >> 4) & 1;
        vm->state.a = __gb_read(&vm->memory, vm->state._sp++);
        break;
    }

    case 0xF2: /* LD A, (C) */
        vm->state.a = __gb_read(&vm->memory, 0xFF00 | vm->state.c);
        break;

    case 0xF3: /* DI */
        vm->state.ime = 0;
        break;

    case 0xF5: /* PUSH AF */
        gb_memory_write(&vm->state, --vm->state._sp, vm->state.a);
        gb_memory_write(&vm->state, --vm->state._sp,
                        vm->state.f_bits.z << 7 | vm->state.f_bits.n << 6 |
                            vm->state.f_bits.h << 5 | vm->state.f_bits.c << 4);
        break;

    case 0xF6: /* OR imm */
        vm->state.a = vm->state.a | __gb_read(&vm->memory, vm->state.pc++);
        vm->state.f_bits.z = (vm->state.a == 0x00);
        vm->state.f_bits.n = 0;
        vm->state.f_bits.h = 0;
        vm->state.f_bits.c = 0;
        break;

    case 0xF7: /* PUSH AF */
        gb_memory_write(&vm->state, --vm->state._sp, vm->state.pc >> 8);
        gb_memory_write(&vm->state, --vm->state._sp, vm->state.pc & 0xFF);
        vm->state.pc = 0x0030;
        break;

    case 0xF8: { /* LD HL, SP+/-imm */
        int8_t offset = (int8_t) __gb_read(&vm->memory, vm->state.pc++);
        vm->state.hl = vm->state._sp + offset;
        vm->state.f_bits.z = 0;
        vm->state.f_bits.n = 0;
        vm->state.f_bits.h =
            ((vm->state._sp & 0xF) + (offset & 0xF) > 0xF) ? 1 : 0;
        vm->state.f_bits.c =
            ((vm->state._sp & 0xFF) + (offset & 0xFF) > 0xFF) ? 1 : 0;
        break;
    }

    case 0xF9: /* LD SP, HL */
        vm->state._sp = vm->state.hl;
        break;

    case 0xFA: { /* LD A, (imm) */
        uint16_t addr = __gb_read(&vm->memory, vm->state.pc++);
        addr |= __gb_read(&vm->memory, vm->state.pc++) << 8;
        vm->state.a = __gb_read(&vm->memory, addr);
        break;
    }

    case 0xFB: /* EI */
        vm->state.ime = 1;
        break;

    case 0xFE: { /* CP imm */
        uint8_t temp_8 = __gb_read(&vm->memory, vm->state.pc++);
        uint16_t temp_16 = vm->state.a - temp_8;
        vm->state.f_bits.z = ((temp_16 & 0xFF) == 0x00);
        vm->state.f_bits.n = 1;
        vm->state.f_bits.h = ((vm->state.a ^ temp_8 ^ temp_16) & 0x10) ? 1 : 0;
        vm->state.f_bits.c = (temp_16 & 0xFF00) ? 1 : 0;
        break;
    }

    case 0xFF: /* RST 0x0038 */
        gb_memory_write(&vm->state, --vm->state._sp, vm->state.pc >> 8);
        gb_memory_write(&vm->state, --vm->state._sp, vm->state.pc & 0xFF);
        vm->state.pc = 0x0038;
        break;
    }
    vm->state.f_subtract = vm->state.f_bits.n;
}
