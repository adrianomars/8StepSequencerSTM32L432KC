#include <stm32l432xx.h>
int read_electronic_signature(SPI_TypeDef *spi,uint8_t *sig,uint32_t sig_len);
int write_enable(SPI_TypeDef *spi);
int write_disable(SPI_TypeDef *spi);
int read_status_register(SPI_TypeDef *spi,uint8_t *status);
int write_status_register(SPI_TypeDef *spi,uint8_t status);
int power_down(SPI_TypeDef *spi);
int power_up(SPI_TypeDef *spi);
int read_data(SPI_TypeDef *spi,uint32_t address, uint8_t *data, uint32_t len);
int page_program(SPI_TypeDef *spi,uint32_t address, uint8_t *data, uint32_t len); // program page.  8LSB's of address should be 0.  len <= 256;
int sector_erase(SPI_TypeDef *spi,uint32_t sector_address);
int block_erase(SPI_TypeDef *spi,uint32_t block_address);
int chip_erase(SPI_TypeDef *spi);
int busy(SPI_TypeDef *spi);
void init_mx25l8005();
void enable_mx25(void);
void disable_mx25(void);