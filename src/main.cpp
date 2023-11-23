#include <stdint.h>

#include <thread>
#include <chrono>

using Clock = std::chrono::steady_clock;

#define RG_R0 0
#define RG_R1 1
#define RG_CX 2
#define RG_TMP 3

#define OP_ADD 0
#define OP_SUB 1
#define OP_LD0 2
#define OP_SWP 3
#define OP_MRD 4
#define OP_MWR 5
#define OP_CPX 6
#define OP_JMP 7
#define OP_NOP 8
#define OP_AND 9
#define OP_BOR 10
#define OP_XOR 11
#define OP_NOT 12
#define OP_HLT 13

#define ALU_NOP 0
#define ALU_ADD 1
#define ALU_SUB 2
#define ALU_AND 3
#define ALU_OR  4
#define ALU_XOR 5
#define ALU_NOT 6

#define MA_IP 0
#define MA_UD 1

/* Microcode layout: */
/*
  bit   0         write ALU regs
  bit   1         ALU reg select
  bits  2 - 4     ALU operation
  bit   5         read reg
  bit   6         write reg
  bits  7 - 8     reg read select
  bits  9 - 10    reg write select
  bit   11        mem io
  bit   12        0 = read mem; 1 = write
  bit   13 - 14   mem address source
  bit   15        ++IP    NOTE: Incriments after cycle
  bit   16        write user defined mem address
  bit   17        0 = write low UD mem address; 1 = write high
  bit   18        copy UDMA => IP
  bit   19        write opcode
  bit   20        reset stage
*/

#define ALU_WRITE(reg) ( 1 | ((reg & 1) << 1) )
#define ALU_DO(op) ( (op & 0b111) << 2 )
#define REG_READ(reg) ( (1 << 5) | ((reg & 0b11) << 7) )
#define REG_WRITE(reg) ( (1 << 6) | ((reg & 0b11) << 9) )
#define MEM_READ(addr) ( (1 << 11) | ((addr & 0b11) << 13) )
#define MEM_WRITE(addr) ( (1 << 11) | (1 << 12) | ((addr & 0b11) << 13) )
#define INCIP ( 1 << 15 )
#define UDMA_WRITEL ( 1 << 16 )
#define UDMA_WRITEH ( (1 << 16) | (1 << 17) )
#define UDMA_TO_IP ( 1 << 18 )
#define OP_WRITE ( 1 << 19 )
#define STAGE_RESET ( 1 << 20 )

#define _MC_GET_BITMASK(ctrl, pos, mask) ( (ctrl & (mask << pos)) >> pos )
#define _MC_GET_BIT(ctrl, bit) _MC_GET_BITMASK(ctrl, bit, 1)

#define MC_ALU_IO(ctrl)           _MC_GET_BIT(ctrl, 0)
#define MC_ALU_REG_SELECT(ctrl)   _MC_GET_BIT(ctrl, 1)
#define MC_ALU_OP(ctrl)           _MC_GET_BITMASK(ctrl, 2, 0b111)
#define MC_REG_READ(ctrl)         _MC_GET_BIT(ctrl, 5)
#define MC_REG_WRITE(ctrl)        _MC_GET_BIT(ctrl, 6)
#define MC_REG_READ_SELECT(ctrl)  _MC_GET_BITMASK(ctrl, 7, 0b11)
#define MC_REG_WRITE_SELECT(ctrl) _MC_GET_BITMASK(ctrl, 9, 0b11)
#define MC_MEM_IO(ctrl)           _MC_GET_BIT(ctrl, 11)
#define MC_MEM_RW(ctrl)           _MC_GET_BIT(ctrl, 12)
#define MC_MEM_ADDRSRC(ctrl)      _MC_GET_BITMASK(ctrl, 13, 0b11)
#define MC_IPINC(ctrl)            _MC_GET_BIT(ctrl, 15)
#define MC_UDMA_IO(ctrl)          _MC_GET_BIT(ctrl, 16)
#define MC_UDMA_HL(ctrl)          _MC_GET_BIT(ctrl, 17)
#define MC_UDMA_TO_IP(ctrl)       _MC_GET_BIT(ctrl, 18)
#define MC_OP_WRITE(ctrl)         _MC_GET_BIT(ctrl, 19)
#define MC_STAGE_RESET(ctrl)      _MC_GET_BIT(ctrl, 20)

// 'shortcut'
#define FETCH_NEXT ( MEM_READ(MA_IP) | OP_WRITE | STAGE_RESET | INCIP )

class MicrocodeProgrammer {
public:
  MicrocodeProgrammer(uint32_t* dest, size_t maxStages) {
    m_dest = dest;
    m_maxStages = maxStages;
  }

public:
  void program() {
    op(OP_ADD)[0] = REG_READ(RG_R0) | ALU_WRITE(0);
    op(OP_ADD)[1] = REG_READ(RG_R1) | ALU_WRITE(1);
    op(OP_ADD)[2] = REG_WRITE(RG_R0) | ALU_DO(ALU_ADD);
    op(OP_ADD)[3] = FETCH_NEXT;

    op(OP_SUB)[0] = REG_READ(RG_R0) | ALU_WRITE(0);
    op(OP_SUB)[1] = REG_READ(RG_R1) | ALU_WRITE(1);
    op(OP_SUB)[2] = REG_WRITE(RG_R0) | ALU_DO(ALU_SUB);
    op(OP_SUB)[3] = FETCH_NEXT;

    op(OP_LD0)[0] = MEM_READ(MA_IP) | REG_WRITE(RG_R0) | INCIP;
    op(OP_LD0)[1] = FETCH_NEXT;

    op(OP_SWP)[0] = REG_WRITE(RG_TMP) | REG_READ(RG_R0);
    op(OP_SWP)[1] = REG_WRITE(RG_R0) | REG_READ(RG_R1);
    op(OP_SWP)[2] = REG_WRITE(RG_R1) | REG_READ(RG_TMP);
    op(OP_SWP)[3] = FETCH_NEXT;

    op(OP_MRD)[0] = MEM_READ(MA_IP) | UDMA_WRITEH | INCIP;
    op(OP_MRD)[1] = MEM_READ(MA_IP) | UDMA_WRITEL | INCIP;
    op(OP_MRD)[2] = MEM_READ(MA_UD) | REG_WRITE(RG_R0);
    op(OP_MRD)[3] = FETCH_NEXT;

    op(OP_MWR)[0] = MEM_READ(MA_IP) | UDMA_WRITEH | INCIP;
    op(OP_MWR)[1] = MEM_READ(MA_IP) | UDMA_WRITEL | INCIP;
    op(OP_MWR)[2] = MEM_WRITE(MA_UD) | REG_READ(RG_R0);
    op(OP_MWR)[3] = FETCH_NEXT;

    op(OP_CPX)[0] = REG_READ(RG_R0) | REG_WRITE(RG_CX);
    op(OP_CPX)[1] = FETCH_NEXT;

    op(OP_JMP)[0] = MEM_READ(MA_IP) | UDMA_WRITEL | INCIP;
    op(OP_JMP)[1] = MEM_READ(MA_IP) | UDMA_WRITEH | INCIP;
    op(OP_JMP)[2] = UDMA_TO_IP;
    op(OP_JMP)[3] = FETCH_NEXT;

    op(OP_NOP)[0] = FETCH_NEXT;

    op(OP_AND)[0] = REG_READ(RG_R0) | ALU_WRITE(0);
    op(OP_AND)[1] = REG_READ(RG_R1) | ALU_WRITE(1);
    op(OP_AND)[2] = REG_WRITE(RG_R0) | ALU_DO(ALU_AND);
    op(OP_AND)[3] = FETCH_NEXT;

    op(OP_BOR)[0] = REG_READ(RG_R0) | ALU_WRITE(0);
    op(OP_BOR)[1] = REG_READ(RG_R1) | ALU_WRITE(1);
    op(OP_BOR)[2] = REG_WRITE(RG_R0) | ALU_DO(ALU_OR);
    op(OP_BOR)[3] = FETCH_NEXT;

    op(OP_XOR)[0] = REG_READ(RG_R0) | ALU_WRITE(0);
    op(OP_XOR)[1] = REG_READ(RG_R1) | ALU_WRITE(1);
    op(OP_XOR)[2] = REG_WRITE(RG_R0) | ALU_DO(ALU_XOR);
    op(OP_XOR)[3] = FETCH_NEXT;

    op(OP_NOT)[0] = REG_READ(RG_R0) | ALU_WRITE(0);
    op(OP_NOT)[1] = REG_WRITE(RG_R0) | ALU_DO(ALU_NOT);
    op(OP_NOT)[2] = FETCH_NEXT;

    op(OP_HLT)[0] = STAGE_RESET;
  }

  uint32_t* op(uint8_t opcode) {
    return &m_dest[opcode * m_maxStages];
  }

private:
  uint32_t* m_dest;
  size_t m_maxStages;
};

class Addressable {
public:
  virtual uint8_t read(uint16_t address) = 0;
  virtual void write(uint16_t address, uint8_t value) = 0;
};

class Memory : public Addressable {
public:
  Memory(size_t kbs, bool writeable, uint8_t* data = nullptr, size_t dataSize = 0) {
    m_size = kbs * 1024;
    m_writeable = writeable;
    m_memory = new uint8_t[m_size];

    if (data)
      memcpy(m_memory, data, dataSize);
  }
  
public:
  bool accessable(uint16_t address) {
    return address < m_size;
  }

  uint8_t read(uint16_t address) {
    if (!accessable(address))
      return 0;

    return m_memory[address];
  }

  void write(uint16_t address, uint8_t value) {
    if (!m_writeable)
      return;

    if (!accessable(address))
      return;

    m_memory[address] = value;
  }

  void setWriteable(bool value) {
    m_writeable = value;
  }

private:
  uint8_t* m_memory;
  size_t m_size;
  bool m_writeable;
};

/*
  Addresses layout:
    0000 - 7FFF : RAM (32K)
    8000 - FFFF : ROM (32k)
*/
class AddressSpace : public Addressable {
public:
  AddressSpace()
    : RAM(32, true), ROM(32, true)
  {
    /* see fib.asm */
    write(0xFF00, 0x02);
    write(0xFF01, 0x00);
    write(0xFF02, 0x03);
    write(0xFF03, 0x02);
    write(0xFF04, 0x01);
    write(0xFF05, 0x03);
    write(0xFF06, 0x00);
    write(0xFF07, 0x03);
    write(0xFF08, 0x07);
    write(0xFF09, 0x06);
    write(0xFF0A, 0xFF);

    ROM.setWriteable(false);
  }

public:
  uint8_t read(uint16_t address) {
    // Yooo, is it really CXX???
    auto [ device, translatedAddress ] = findDevice(address);

    return device.read(translatedAddress);
  }

  void write(uint16_t address, uint8_t value) {
    auto [ device, translatedAddress ] = findDevice(address);

    device.write(translatedAddress, value);
  }

private:
  std::pair<Addressable&, uint16_t> findDevice(uint16_t address) {
    if (address & 0x8000)
      return { ROM, address & 0x7FFF };
    else
      return { RAM, address };
  }

private:
  Memory RAM;
  Memory ROM;
};

class Cycleable {
public:
  // Output from this unit
  // We need to output definitially before any other unit will read our data
  virtual void cycle0(uint32_t control) = 0;

  // Input from data bus
  virtual void cycle1(uint32_t control) = 0;
};

class ALU : public Cycleable {
public:
  ALU(uint8_t* dataBus)
    : DataBus(dataBus) {}

public:
  void cycle0(uint32_t control) {
    uint32_t op = MC_ALU_OP(control);

    switch (op) {
      default:
        break;
      case ALU_ADD:
        *DataBus = Reg0 + Reg1;
        printf("[ALU] ADD(%02Xh, %02Xh) => %02Xh\n", Reg0, Reg1, *DataBus);
        break;
      case ALU_SUB:
        *DataBus = Reg0 - Reg1;
        printf("[ALU] SUB(%02Xh, %02Xh) => %02Xh\n", Reg0, Reg1, *DataBus);
        break;
      case ALU_AND:
        *DataBus = Reg0 & Reg1;
        printf("[ALU] AND(%02Xh, %02Xh) => %02Xh\n", Reg0, Reg1, *DataBus);
        break;
      case ALU_OR:
        *DataBus = Reg0 | Reg1;
        printf("[ALU] OR(%02Xh, %02Xh) => %02Xh\n", Reg0, Reg1, *DataBus);
        break;
      case ALU_XOR:
        *DataBus = Reg0 ^ Reg1;
        printf("[ALU] XOR(%02Xh, %02Xh) => %02Xh\n", Reg0, Reg1, *DataBus);
        break;
      case ALU_NOT:
        *DataBus = ~Reg0;
        printf("[ALU] NOT(%02Xh) => %02Xh\n", Reg0, *DataBus);
        break;
    }
  }

  void cycle1(uint32_t control) {
    if (MC_ALU_IO(control)) {
      uint32_t regSelect = MC_ALU_REG_SELECT(control);

      printf("[ALU] Write Reg%d = %02Xh\n", regSelect, *DataBus);

      if (regSelect)
        Reg1 = *DataBus;
      else
        Reg0 = *DataBus;
    }
  }

private:
  uint8_t Reg0 = 0;
  uint8_t Reg1 = 0;

  uint8_t* DataBus;
};

class Registers : public Cycleable {
public:
  Registers(uint8_t* dataBus)
    : DataBus(dataBus) {}

public:
  void cycle0(uint32_t control) {
    // read
    if (MC_REG_READ(control)) {
      uint32_t regSelect = MC_REG_READ_SELECT(control);

      switch (regSelect) {
        default:
          break;
        case RG_R0:
          printf("[REG] Read R0 => %02Xh\n", R0);
          *DataBus = R0;
          break;
        case RG_R1:
          printf("[REG] Read R1 => %02Xh\n", R1);
          *DataBus = R1;
          break;
        case RG_CX:
          printf("[REG] Read CX => %02Xh\n", CX);
          *DataBus = CX;
          break;
        case RG_TMP:
          printf("[REG] Read TMP => %02Xh\n", TMP);
          *DataBus = TMP;
          break;
      }
    }
  }

  void cycle1(uint32_t control) {
    // write
    if (MC_REG_WRITE(control)) {
      uint32_t regSelect = MC_REG_WRITE_SELECT(control);

      switch (regSelect) {
        case RG_R0:
          R0 = *DataBus;
          printf("[REG] Write R0 = %02Xh\n", R0);
          break;
        case RG_R1:
          R1 = *DataBus;
          printf("[REG] Write R1 = %02Xh\n", R1);
          break;
        case RG_CX:
          CX = *DataBus;
          printf("[REG] Write CX = %02Xh\n", CX);
          break;
        case RG_TMP:
          TMP = *DataBus;
          printf("[REG] Write TMP = %02Xh\n", TMP);
          break;
      }
    }
  }

private:
  uint8_t R0 = 0;
  uint8_t R1 = 0;
  uint8_t CX = 0;
  uint8_t TMP = 0;

  uint8_t* DataBus;
};

class MemoryInterface : public Cycleable {
public:
  MemoryInterface(uint8_t* dataBus, AddressSpace* addrSpace)
    : DataBus(dataBus), AddrSpace(addrSpace) {}

public:
  void cycle0(uint32_t control) {
    if (MC_UDMA_TO_IP(control)) {
      IP = UD;
      printf("[MEM] IP = UD (%04Xh)\n", UD);
    }

    if (MC_MEM_IO(control) && MC_MEM_RW(control) == 0) {
      uint16_t addr = 0;
      
      const char* usedReg;

      switch (MC_MEM_ADDRSRC(control)) {
        default:
          addr = 0;
          usedReg = "<NONE>";
          break;
        case MA_IP:
          addr = IP;
          usedReg = "IP";
          break;
        case MA_UD:
          addr = UD;
          usedReg = "UD";
          break;
      }

      *DataBus = AddrSpace->read(addr);
      printf("[MEM] Reading %04Xh (from %s) : %02Xh\n", addr, usedReg, *DataBus);
    }
  }

  void cycle1(uint32_t control) {
    if (MC_UDMA_IO(control)) {
      if (MC_UDMA_HL(control)) {
        // high
        UD &= 0x00FF;
        UD |= (*DataBus) << 8;
        printf("[MEM] UD high byte write %02Xh (UD = %04Xh)\n", *DataBus, UD);
      }
      else {
        // low
        UD &= 0xFF00;
        UD |= *DataBus;
        printf("[MEM] UD low byte write %02Xh (UD = %04Xh)\n", *DataBus, UD);
      }
    }

    if (MC_MEM_IO(control) && MC_MEM_RW(control)) {
      uint16_t addr = 0;

      const char* usedReg;

      switch (MC_MEM_ADDRSRC(control)) {
        case MA_IP:
          addr = IP;
          usedReg = "IP";
          break;
        case MA_UD:
          addr = UD;
          usedReg = "UD";
          break;
      }

      AddrSpace->write(addr, *DataBus);
      printf("[MEM] Writing %02Xh to %04Xh (from %s)\n", *DataBus, addr, usedReg);
    }

    if (MC_IPINC(control)) {
      ++IP;
      printf("[MEM] ++IP (now %04Xh)\n", IP);
    }
  }

private:
  uint16_t IP = 0xFF00;
  uint16_t UD = 0;

  AddressSpace* AddrSpace;
  uint8_t* DataBus;
};

class CPU {
public:
  CPU(AddressSpace* addrSpace)
    : ALU(&DataBus), Regs(&DataBus),
      Mem(&DataBus, addrSpace) {}

public:
  void start() {
    MicrocodeProgrammer programmer = MicrocodeProgrammer( (uint32_t*)Microcode, 4 );
    programmer.program();

    Opcode = OP_NOP;
    Stage = 0;

    while (!m_shutdown) {
      Clock::time_point nextClock
        = Clock::now() + std::chrono::milliseconds(100);

      cycle();

      std::this_thread::sleep_until(nextClock);
    }
  }

private:
  void cycle() {
    uint32_t controlBus = Microcode[Opcode][Stage];
    DataBus = 0;

    ALU.cycle0(controlBus);
    Regs.cycle0(controlBus);
    Mem.cycle0(controlBus);

    ALU.cycle1(controlBus);
    Regs.cycle1(controlBus);
    Mem.cycle1(controlBus);

    ++Stage;

    if (MC_OP_WRITE(controlBus)) {
      Opcode = DataBus;
      printf("[CPU] Opcode = %02Xh\n", Opcode);
    }

    if (MC_STAGE_RESET(controlBus)) {
      Stage = 0;
      printf("[CPU] Stage Reset\n", Opcode);
    }

    printf("\n");
  }

private:
  uint8_t Opcode;
  uint8_t Stage;
  uint8_t DataBus;

  uint32_t Microcode[64][4];

  ALU ALU;
  Registers Regs;
  MemoryInterface Mem;

private:
  bool m_shutdown;
};

int main() {
  AddressSpace addrSpace;
  CPU vm = CPU(&addrSpace);

  vm.start();

  return 0;
}
