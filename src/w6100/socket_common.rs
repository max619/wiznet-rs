use crate::w6100::{
    Error,
    socket::SocketProtocolMode,
    transiver::{Address, BlockAddress, Transceiver},
};

use bitflags::bitflags;

/// (SOCKET n Mode Register) R=W 0x00
const SN_MR: u16 = 0x0000;

/// (SOCKET n Prefer Source IPv6 Address Register) R=W 0x00
const SN_PSR: u16 = 0x0004;

/// (SOCKET n Command Register) RW,AC 0x00
const SN_CR: u16 = 0x0010;

/// (SOCKET n Interrupt Register) WO 0x00
const SN_IR: u16 = 0x0020;

/// (SOCKET n Interrupt Mask Register) R=W 0xFF
const SN_IMR: u16 = 0x0024;

/// (Sn_IR Clear Register) WO 0xFF
const SN_IRCLR: u16 = 0x0028;

/// (SOCKET n Status Register) RO 0x00
const SN_SR: u16 = 0x0030;

/// (SOCKET n Extension Status Register) RO 0x00
const SN_ESR: u16 = 0x0031;

/// (SOCKET n IP Protocol Number Register) R=W 0x00
const SN_PNR: u16 = 0x0100;

/// (SOCKET n IP Type Of Service Register) R=W 0x00
const SN_TOSR: u16 = 0x0104;

/// (SOCKET n IP Time To Live Register) R=W 0x80
const SN_TTLR: u16 = 0x0108;

/// (SOCKET n Fragment Offset in IP Header Register) R=W 0x40
const SN_FRGR0: u16 = 0x010C;

/// R=W 0x00
const SN_FRGR1: u16 = 0x010D;

/// (SOCKET n Maximum Segment Size Register) RW 0x00
const SN_MSSR0: u16 = 0x0110;

/// RW 0x00
const SN_MSSR1: u16 = 0x0111;

/// (SOCKET n Source Port Register) R=W 0x00
const SN_PORTR0: u16 = 0x0114;

/// R=W 0x00
const SN_PORTR1: u16 = 0x0115;

/// (SOCKET n Destination Hardware Address Register) RW 0x00
const SN_DHAR0: u16 = 0x0118;

/// RW 0x00
const SN_DHAR1: u16 = 0x0119;

/// RW 0x00
const SN_DHAR2: u16 = 0x011A;

/// RW 0x00
const SN_DHAR3: u16 = 0x011B;

/// RW 0x00
const SN_DHAR4: u16 = 0x011C;

/// RW 0x00
const SN_DHAR5: u16 = 0x011D;

/// (SOCKET n Destination IPv4 Address Register) RW 0x00
const SN_DIPR0: u16 = 0x0120;

/// RW 0x00
const SN_DIPR1: u16 = 0x0121;

/// RW 0x00
const SN_DIPR2: u16 = 0x0122;

/// RW 0x00
const SN_DIPR3: u16 = 0x0123;

/// (SOCKET n Destination IPv6 Address Register) RW 0x00
const SN_DIP6R0: u16 = 0x0130;

/// RW 0x00
const SN_DIP6R1: u16 = 0x0131;

/// RW 0x00
const SN_DIP6R2: u16 = 0x0132;

/// RW 0x00
const SN_DIP6R3: u16 = 0x0133;

/// RW 0x00
const SN_DIP6R4: u16 = 0x0134;

/// RW 0x00
const SN_DIP6R5: u16 = 0x0135;

/// RW 0x00
const SN_DIP6R6: u16 = 0x0136;

/// RW 0x00
const SN_DIP6R7: u16 = 0x0137;

/// RW 0x00
const SN_DIP6R8: u16 = 0x0138;

/// RW 0x00
const SN_DIP6R9: u16 = 0x0139;

/// RW 0x00
const SN_DIP6R10: u16 = 0x013A;

/// RW 0x00
const SN_DIP6R11: u16 = 0x013B;

/// RW 0x00
const SN_DIP6R12: u16 = 0x013C;

/// RW 0x00
const SN_DIP6R13: u16 = 0x013D;

/// RW 0x00
const SN_DIP6R14: u16 = 0x013E;

/// RW 0x00
const SN_DIP6R15: u16 = 0x013F;

/// (SOCKET n Destination Port Register) RW 0x00
const SN_DPORTR0: u16 = 0x0140;

/// RW 0x00
const SN_DPORTR1: u16 = 0x0141;

/// (SOCKET n Mode Register 2) R=W 0x00
const SN_MR2: u16 = 0x0144;

/// (SOCKET n Retransmission Time Register) RW 0x00
const SN_RTR0: u16 = 0x0180;

/// RW 0x00
const SN_RTR1: u16 = 0x0181;

/// (SOCKET n Retransmission Count Register) RW 0x00
const SN_RCR: u16 = 0x0184;

/// (SOCKET n Keep Alive Time Register) R=W 0x00
const SN_KPALVTR: u16 = 0x0188;

/// (SOCKET n TX Buffer Size Register) R=W 0x02
const SN_TX_BSR: u16 = 0x0200;

/// (SOCKET n TX Free Size Register) RO 0x00
const SN_TX_FSR0: u16 = 0x0204;

/// RO 0x00
const SN_TX_FSR1: u16 = 0x0205;

/// (SOCKET n TX Read Pointer Register) RO 0x00
const SN_TX_RD0: u16 = 0x0208;

/// RO 0x00
const SN_TX_RD1: u16 = 0x0209;

/// (SOCKET n TX Write Pointer Register) RW 0x00
const SN_TX_WR0: u16 = 0x020C;

/// RW 0x00
const SN_TX_WR1: u16 = 0x020D;

/// (SOCKET n RX Buffer Size Register) R=W 0x02
const SN_RX_BSR: u16 = 0x0220;

/// (SOCKET n RX Received Size Register) RO 0x00
const SN_RX_RSR0: u16 = 0x0224;

/// RO 0x00
const SN_RX_RSR1: u16 = 0x0225;

/// (SOCKET n RX Read Pointer Register) RW 0x00
const SN_RX_RD0: u16 = 0x0228;

/// RW 0x00
const SN_RX_RD1: u16 = 0x0229;

/// (SOCKET n RX Write Pointer Register) RO 0x00
const SN_RX_WR0: u16 = 0x022C;

/// RO 0x00
const SN_RX_WR1: u16 = 0x022D;

bitflags! {
    #[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
    pub(crate) struct SocketInterrupt: u8 {
        const SENDOK = 1 << 4;
        const TIMEOUT = 1 << 3;
        const RECV = 1 << 2;
        const DISCON = 1 << 1;
        const CON = 1 << 0;
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub(crate) enum SocketCommand {
    None = 0,

    Open = 0x01,
    Listen = 0x02,

    Connect = 0x04,
    Connect6 = 0x84,

    Disconnect = 0x8,

    Close = 0x10,

    Send = 0x20,
    Send6 = 0xA0,

    SendKeep = 0x22,

    Receive = 0x40,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub(crate) enum SocketStatusRegister {
    // SOCKET n closed.
    Closed = 0x00,
    // SOCKET n opened in TCP Mode.
    Init = 0x13,
    // SOCKET n is in TCP Mode and waits for Connection request.
    Listen = 0x14,
    // SOCKET n is in TCP Mode and TCP Connection is completed.
    Established = 0x17,
    // SOCKET n is in TCP Mode and received FIN Packet.
    CloseWait = 0x1C,
    // SOCKET n opened in UDP Mode.
    Udp = 0x22,
    // SOCKET n opened in IPRAW Mode.
    Ipraw = 0x32,
    // SOCKET n opened in IPRAW6 Mode.
    Ipraw6 = 0x33,
    // SOCKET n opened in MACRAW Mode.
    Macraw = 0x42,
    // The status of sending Connect-Request.
    Synsent = 0x15,
    // The status of receiving Connect-Request.
    Synrecv = 0x16,

    ///The status of closing SOCKET n.
    FinWait = 0x18,
    ///The status of closing SOCKET n.
    TimeWait = 0x1B,
    ///The status of closing SOCKET n.
    LastAck = 0x1D,

    Unknown = 0xFF,
}

pub(crate) fn init_socket<Trans: Transceiver>(
    block: &BlockAddress,
    trans: &mut Trans,
    mode: SocketProtocolMode,
) -> Result<(), Error> {
    trans.write_u8(
        &Address {
            address: SN_MR,
            block: block.reg,
        },
        mode.bits(),
    )?;

    // Clear interrupts
    trans.write_u8(
        &Address {
            address: SN_IRCLR,
            block: block.reg,
        },
        0xFF,
    )?;

    // Enable all interrupts
    trans.write_u8(
        &Address {
            address: SN_IMR,
            block: block.reg,
        },
        0xFF,
    )?;

    Ok(())
}

pub(crate) fn get_interrupts<Trans: Transceiver>(
    block: &BlockAddress,
    trans: &mut Trans,
) -> Result<SocketInterrupt, Error> {
    let ir = trans.read_u8(&Address {
        address: SN_IR,
        block: block.reg,
    })?;

    Ok(SocketInterrupt::from_bits_truncate(ir))
}

pub(crate) fn clear_interrupts<Trans: Transceiver>(
    block: &BlockAddress,
    trans: &mut Trans,
    mask: SocketInterrupt,
) -> Result<(), Error> {
    trans.write_u8(
        &Address {
            address: SN_IRCLR,
            block: block.reg,
        },
        mask.bits(),
    )
}

pub(crate) fn set_src_port<Trans: Transceiver>(
    block: &BlockAddress,
    trans: &mut Trans,
    port: u16,
) -> Result<(), Error> {
    trans.write_u16(
        &Address {
            address: SN_PORTR0,
            block: block.reg,
        },
        port,
    )
}

pub(crate) fn set_dst_port<Trans: Transceiver>(
    block: &BlockAddress,
    trans: &mut Trans,
    port: u16,
) -> Result<(), Error> {
    trans.write_u16(
        &Address {
            address: SN_DPORTR0,
            block: block.reg,
        },
        port,
    )
}

pub(crate) fn set_ipv4_dst_addr<Trans: Transceiver>(
    block: &BlockAddress,
    trans: &mut Trans,
    address: u32,
) -> Result<(), Error> {
    trans.write_u32(
        &Address {
            address: SN_DIPR0,
            block: block.reg,
        },
        address,
    )
}

pub(crate) fn send_sock_command<Trans: Transceiver>(
    block: &BlockAddress,
    trans: &mut Trans,
    command: SocketCommand,
) -> Result<(), Error> {
    trans.write_u8(
        &Address {
            address: SN_CR,
            block: block.reg,
        },
        command as u8,
    )
}

/// Number of bytes currently waiting in the SOCKET's RX buffer on the chip.
pub(crate) fn get_rx_received_size<Trans: Transceiver>(
    block: &BlockAddress,
    trans: &mut Trans,
) -> Result<u16, Error> {
    trans.read_u16(&Address {
        address: SN_RX_RSR0,
        block: block.reg,
    })
}

/// Current RX read pointer (Sn_RX_RD) — a free-running 16-bit offset.
pub(crate) fn get_rx_read_pointer<Trans: Transceiver>(
    block: &BlockAddress,
    trans: &mut Trans,
) -> Result<u16, Error> {
    trans.read_u16(&Address {
        address: SN_RX_RD0,
        block: block.reg,
    })
}

/// Advance the RX read pointer (Sn_RX_RD) after consuming data.
pub(crate) fn set_rx_read_pointer<Trans: Transceiver>(
    block: &BlockAddress,
    trans: &mut Trans,
    pointer: u16,
) -> Result<(), Error> {
    trans.write_u16(
        &Address {
            address: SN_RX_RD0,
            block: block.reg,
        },
        pointer,
    )
}

/// Burst-read `data.len()` bytes out of the SOCKET's RX buffer starting at the
/// given pointer. The chip auto-increments and wraps within the buffer region.
pub(crate) fn read_rx_buffer<Trans: Transceiver>(
    block: &BlockAddress,
    trans: &mut Trans,
    pointer: u16,
    data: &mut [u8],
) -> Result<(), Error> {
    trans.read(
        &Address {
            address: pointer,
            block: block.rx,
        },
        data,
    )
}

pub(crate) fn is_command_pending<Trans: Transceiver>(
    block: &BlockAddress,
    trans: &mut Trans,
) -> Result<bool, Error> {
    Ok(trans.read_u8(&Address {
        address: SN_CR,
        block: block.reg,
    })? != 0)
}

pub(crate) fn read_status<Trans: Transceiver>(
    block: &BlockAddress,
    trans: &mut Trans,
) -> Result<SocketStatusRegister, Error> {
    let value = trans.read_u8(&Address {
        address: SN_SR,
        block: block.reg,
    })?;

    Ok(match value {
        0x00 => SocketStatusRegister::Closed,
        0x13 => SocketStatusRegister::Init,
        0x14 => SocketStatusRegister::Listen,
        0x17 => SocketStatusRegister::Established,
        0x1C => SocketStatusRegister::CloseWait,
        0x22 => SocketStatusRegister::Udp,
        0x32 => SocketStatusRegister::Ipraw,
        0x33 => SocketStatusRegister::Ipraw6,
        0x42 => SocketStatusRegister::Macraw,
        0x15 => SocketStatusRegister::Synsent,
        0x16 => SocketStatusRegister::Synrecv,

        0x18 => SocketStatusRegister::FinWait,
        0x1B => SocketStatusRegister::TimeWait,
        0x1D => SocketStatusRegister::LastAck,

        _ => SocketStatusRegister::Unknown,
    })
}
