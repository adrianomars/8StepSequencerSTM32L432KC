#include "MX25L8005.h"
#include "spi.h"
#include "eeng1030_lib.h"
int read_electronic_signature(SPI_TypeDef *spi,uint8_t *sig,uint32_t sig_len)
{
    uint8_t cmd=0x90;
	spi_exchange(spi,&cmd,1,sig,sig_len); // should return the values 0x13 and 0xc2 in locations 4 and 5 in sig	
	return 0;
}
int write_enable(SPI_TypeDef *spi)
{
    uint8_t cmd=0x06;
    uint8_t rxdata[1];
	spi_exchange(spi,&cmd,1,rxdata,0); // should return the values 0xc2 and 0x13 in locations 4 and 5 in sig
	return 0;
}
int write_disable(SPI_TypeDef *spi)
{
    uint8_t cmd=0x04;
    uint8_t rxdata[1];
	spi_exchange(spi,&cmd,1,rxdata,0); // should return the values 0xc2 and 0x13 in locations 4 and 5 in sig
	return 0;  
}
int read_status_register(SPI_TypeDef *spi,uint8_t *status)
{    
	uint8_t cmd=0x05;
	uint8_t rxdata[2];
	int ret;
	spi_exchange(spi,&cmd,1,rxdata,2); // should return the values 0xc2 and 0x13 in locations 4 and 5 in sig
	*status=rxdata[1];
	return 0;
}
int write_status_register(SPI_TypeDef *spi,uint8_t status)
{
    uint8_t txdata[2];
	txdata[0]=0x04;
	txdata[1]=status;	
    uint8_t rxdata[1];
	spi_exchange(spi,txdata,2,rxdata,0); // should return the values 0xc2 and 0x13 in locations 4 and 5 in sig
	return 0;
}
int power_down(SPI_TypeDef *spi)
{
    uint8_t cmd=0xb9;
    uint8_t rxdata[1];
	spi_exchange(spi,&cmd,1,rxdata,0); // should return the values 0xc2 and 0x13 in locations 4 and 5 in sig
	return 0;
}
int power_up(SPI_TypeDef *spi)
{
    uint8_t cmd=0xab;
    uint8_t rxdata[1];
	spi_exchange(spi,&cmd,1,rxdata,0); // should return the values 0xc2 and 0x13 in locations 4 and 5 in sig
	return 0;
}
int read_data(SPI_TypeDef *spi,uint32_t address, uint8_t *data, uint32_t len)
{
	uint8_t txdata[4];
	txdata[0]=0x03;
	txdata[1]=address>>16;	
	txdata[2]=(address >> 8) & 0xff;
	txdata[3]=address & 0xff;	
    spi_exchange(spi,txdata,4,data,len);	
	return 0;
}

int page_program(SPI_TypeDef *spi,uint32_t address, uint8_t *data, uint32_t len) // program page.  8LSB's of address should be 0.  len <= 256;
{
    uint8_t txdata[4+len];
    uint8_t rxdata[1];
    volatile uint32_t busycount=0;
	txdata[0]=0x02;
	txdata[1]=address>>16;	
	txdata[2]=(address >> 8) & 0xff;
	txdata[3]=address & 0xff;
    for (uint32_t i=0;i<len;i++)
    {
        txdata[4+i]=data[i];
    }
    spi_exchange(spi,txdata,4+len,rxdata,0);	
	while (busy(spi))
	{
		busycount++;
	}
	return 0;
}
int sector_erase(SPI_TypeDef *spi,uint32_t sector_address)
{
    uint8_t txdata[5];
    uint8_t rxdata[1];
    volatile uint32_t busycount=0;

	txdata[0]=0x20;
	txdata[1]=sector_address>>16;	
	txdata[2]=(sector_address >> 8) & 0xff;
	txdata[3]=0;
    spi_exchange(spi,txdata,4,rxdata,0);		
	while (busy(spi))
	{
		busycount++;
	}
	return 0;
}
int block_erase(SPI_TypeDef *spi,uint32_t block_address)
{
    uint8_t txdata[5];
    uint8_t rxdata[1];
    volatile uint32_t busycount=0;

    txdata[0]=0x52;
	txdata[1]=block_address>>16;	
	txdata[2]=(block_address >> 8) & 0xff;
	txdata[3]=0;
    spi_exchange(spi,txdata,4,rxdata,0);		
	while (busy(spi))
	{
		busycount++;
	}
	return 0;
}
int chip_erase(SPI_TypeDef *spi)
{
    uint8_t txdata[5];
    uint8_t rxdata[1];
    volatile uint32_t busycount=0;
    txdata[0]=0x60;	
    spi_exchange(spi,txdata,1,rxdata,0);		
	while (busy(spi))
	{
		busycount++;
	}
	return 0;
}
int busy(SPI_TypeDef *spi)
{
// Must now wait for write to complete.
// Poll the Write In Progess bit in the status register (LSB)
	uint8_t stat;
	read_status_register(spi,&stat);
	return (stat & 1);
}
void enable_mx25(void)
{
	GPIOB->ODR &= ~(1 << 0); // drive PB0 (CS for MX flash chip) low
}
void disable_mx25(void)
{
	GPIOB->ODR |= (1 << 0); // drive PB0 (CS for MX flash chip) high

}
void init_mx25l8005()
{
	// assuming SPI is initialized elsewhere
	pinMode(GPIOB,0,1);
    GPIOB->ODR |= (1<<0);
}