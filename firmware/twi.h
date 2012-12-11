int twiReadReg (const uint8_t addr, const uint8_t reg);
int twiReadRegN (const uint8_t addr, const uint8_t reg, uint8_t n, void *pdata);
int twiWriteReg (const uint8_t addr, const uint8_t reg, const uint8_t data);
int twiWriteRegN (const uint8_t addr, const uint8_t reg, uint8_t n, void *pdata);
void twiInit (void);
