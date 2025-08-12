#!/usr/bin/env python3
"""
MCU-32X Minimal Assembler

This script assembles a subset of MCU-32X assembly into 32-bit machine code words.
Supported instructions: ADD, SUB, AND, OR, XOR, NOR, SLL, SRL, SRA, MUL, DIV, LW, SW, BEQ, J

Usage:
    python mcu32x_assembler.py program.s > program.hex
"""
import sys
import re

OPCODES = {
    'R':    0b000000,
    'LW':   0b100011,
    'SW':   0b101011,
    'BEQ':  0b000100,
    'J':    0b000010,
}
FUNCTS = {
    'ADD':  0b0000,
    'SUB':  0b0001,
    'AND':  0b0010,
    'OR':   0b0011,
    'XOR':  0b0100,
    'NOR':  0b0101,
    'SLL':  0b0110,
    'SRL':  0b0111,
    'SRA':  0b1000,
    'MUL':  0b1001,
    'DIV':  0b1010,
}

def regnum(r):
    if isinstance(r, int):
        return r
    if r.startswith('x'):
        return int(r[1:])
    raise ValueError(f'Unknown register: {r}')

def assemble_rtype(mnemonic, rd, rs, rt):
    opcode = OPCODES['R']
    funct = FUNCTS[mnemonic.upper()]
    word = (opcode << 26) | (regnum(rs) << 19) | (regnum(rt) << 20) | (regnum(rd) << 0) | (funct << 0)
    return word

def assemble_itype(mnemonic, rt, rs, imm):
    opcode = OPCODES[mnemonic.upper()]
    imm = int(imm) & 0xFFFF
    word = (opcode << 26) | (regnum(rs) << 19) | (regnum(rt) << 20) | (imm & 0xFFFF)
    return word

def assemble_jtype(addr):
    opcode = OPCODES['J']
    addr = int(addr) & 0x3FFFFFF
    word = (opcode << 26) | addr
    return word

def parse_line(line):
    line = line.split('#')[0].strip()
    if not line:
        return None
    tokens = re.split(r'[\s,()]+', line)
    mnemonic = tokens[0].upper()
    if mnemonic in FUNCTS:
        # R-type: add rd, rs, rt
        rd, rs, rt = tokens[1:4]
        return assemble_rtype(mnemonic, rd, rs, rt)
    elif mnemonic == 'LW' or mnemonic == 'SW':
        # I-type: lw rt, imm(rs)
        rt, imm, rs = tokens[1], tokens[2], tokens[3]
        return assemble_itype(mnemonic, rt, rs, imm)
    elif mnemonic == 'BEQ':
        # I-type: beq rs, rt, imm
        rs, rt, imm = tokens[1:4]
        return assemble_itype(mnemonic, rt, rs, imm)
    elif mnemonic == 'J':
        # J-type: j addr
        addr = tokens[1]
        return assemble_jtype(addr)
    else:
        raise ValueError(f'Unknown instruction: {mnemonic}')

def main():
    for line in sys.stdin:
        try:
            word = parse_line(line)
            if word is not None:
                print(f'{word:08X}')
        except Exception as e:
            print(f'# Error: {e} in line: {line.strip()}', file=sys.stderr)

if __name__ == '__main__':
    main()
