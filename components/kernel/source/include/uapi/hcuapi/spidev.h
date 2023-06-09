#ifndef SPIDEV_H
#define SPIDEV_H

#include <hcuapi/iocbase.h>

#define SPI_CPHA		0x01
#define SPI_CPOL		0x02

#define SPI_MODE_0		(0|0)
#define SPI_MODE_1		(0|SPI_CPHA)
#define SPI_MODE_2		(SPI_CPOL|0)
#define SPI_MODE_3		(SPI_CPOL|SPI_CPHA)

#define SPI_CS_HIGH		0x04
#define SPI_LSB_FIRST		0x08
#define SPI_3WIRE		0x10
#define SPI_LOOP		0x20
#define SPI_NO_CS		0x40
#define SPI_READY		0x80
#define SPI_TX_DUAL		0x100
#define SPI_TX_QUAD		0x200
#define SPI_RX_DUAL		0x400
#define SPI_RX_QUAD		0x800

/*---------------------------------------------------------------------------*/

/* IOCTL commands */



/**
 * struct spi_ioc_transfer - describes a single SPI transfer
 * @tx_buf: Holds pointer to userspace buffer with transmit data, or null.
 *	If no data is provided, zeroes are shifted out.
 * @rx_buf: Holds pointer to userspace buffer for receive data, or null.
 * @len: Length of tx and rx buffers, in bytes.
 * @speed_hz: Temporary override of the device's bitrate.
 * @bits_per_word: Temporary override of the device's wordsize.
 * @delay_usecs: If nonzero, how long to delay after the last bit transfer
 *	before optionally deselecting the device before the next transfer.
 * @cs_change: True to deselect device before starting the next transfer.
 *
 * This structure is mapped directly to the kernel spi_transfer structure;
 * the fields have the same meanings, except of course that the pointers
 * are in a different address space (and may be of different sizes in some
 * cases, such as 32-bit i386 userspace over a 64-bit x86_64 kernel).
 * Zero-initialize the structure, including currently unused fields, to
 * accommodate potential future updates.
 *
 * SPI_IOC_MESSAGE gives userspace the equivalent of kernel spi_sync().
 * Pass it an array of related transfers, they'll execute together.
 * Each transfer may be half duplex (either direction) or full duplex.
 *
 *	struct spi_ioc_transfer mesg[4];
 *	...
 *	status = ioctl(fd, SPI_IOC_MESSAGE(4), mesg);
 *
 * So for example one transfer might send a nine bit command (right aligned
 * in a 16-bit word), the next could read a block of 8-bit data before
 * terminating that command by temporarily deselecting the chip; the next
 * could send a different nine bit command (re-selecting the chip), and the
 * last transfer might write some register values.
 */
struct spi_ioc_transfer {
	uint32_t	tx_buf;
	uint32_t	rx_buf;

	uint32_t	len;
	uint32_t	speed_hz;

	uint16_t	delay_usecs;
	uint8_t		bits_per_word;
	uint8_t		cs_change;
	uint8_t		tx_nbits;
	uint8_t		rx_nbits;
	uint16_t	pad;

	/* If the contents of 'struct spi_ioc_transfer' ever change
	 * incompatibly, then the ioctl number (currently 0) must change;
	 * ioctls with constant size fields get a bit more in the way of
	 * error checking than ones (like this) where that field varies.
	 *
	 * NOTE: struct layout is the same in 64bit and 32bit userspace.
	 */
};

/* not all platforms use <asm-generic/ioctl.h> or _IOC_TYPECHECK() ... */
#define SPI_MSGSIZE(N) \
	((((N)*(sizeof (struct spi_ioc_transfer))) < (1 << _IOC_SIZEBITS)) \
		? ((N)*(sizeof (struct spi_ioc_transfer))) : 0)
#define SPI_IOC_MESSAGE(N) _IOW(SPI_IOCBASE, 0, char[SPI_MSGSIZE(N)])


/* Read / Write of SPI mode (SPI_MODE_0..SPI_MODE_3) (limited to 8 bits) */
#define SPI_IOC_RD_MODE			_IOR(SPI_IOCBASE, 1, uint8_t)
#define SPI_IOC_WR_MODE			_IOW(SPI_IOCBASE, 1, uint8_t)

/* Read / Write SPI bit justification */
#define SPI_IOC_RD_LSB_FIRST		_IOR(SPI_IOCBASE, 2, uint8_t)
#define SPI_IOC_WR_LSB_FIRST		_IOW(SPI_IOCBASE, 2, uint8_t)

/* Read / Write SPI device word length (1..N) */
#define SPI_IOC_RD_BITS_PER_WORD	_IOR(SPI_IOCBASE, 3, uint8_t)
#define SPI_IOC_WR_BITS_PER_WORD	_IOW(SPI_IOCBASE, 3, uint8_t)

/* Read / Write SPI device default max speed hz */
#define SPI_IOC_RD_MAX_SPEED_HZ		_IOR(SPI_IOCBASE, 4, uint32_t)
#define SPI_IOC_WR_MAX_SPEED_HZ		_IOW(SPI_IOCBASE, 4, uint32_t)

/* Read / Write of the SPI mode field */
#define SPI_IOC_RD_MODE32		_IOR(SPI_IOCBASE, 5, uint32_t)
#define SPI_IOC_WR_MODE32		_IOW(SPI_IOCBASE, 5, uint32_t)



#endif /* SPIDEV_H */
