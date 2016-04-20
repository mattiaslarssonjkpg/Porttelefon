using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

/* For SPI drivers */
using Windows.Devices.Spi;
using Windows.Devices.Enumeration;
using Windows.Devices.Gpio;

/* RegisterMap of the RFID reader */
using IoTCore.RFID_Constans;
using System.Diagnostics;

namespace IoTCore {
    class MFRC552 {
        private const Byte DATA_COMMAND_PIN = 22; /* We use GPIO 22 since it's conveniently near the SPI pins */
        private const String READ = "readreg";
        private const String WRITE = "writereg";

        private GpioController _gpioController;
        private SpiDevice _spiDevice;
        private GpioPin _dataCommandPin, _resetPin;

        /**
         * FUNCTION NAME : openMFRC522
         * DESCRIPTION   : Open up SPI bus so that we can start sending and reciving data over the bus
         * INPUT         : -
         * OUTPUT        : - 
         * NOTE          : Futher implementaions might be needed
         */

        public async void openMFRC522() {
            try {
                /* Get defualt Gpiocontroller */
                _gpioController = GpioController.GetDefault();

                /* Initialize a pin as output for the hardware Reset line on the RFID */
                _resetPin = _gpioController.OpenPin(37);
                _resetPin.Write(GpioPinValue.High);
                _resetPin.SetDriveMode(GpioPinDriveMode.Output);

                _dataCommandPin = _gpioController.OpenPin(37);
                _dataCommandPin.Write(GpioPinValue.High);
                _dataCommandPin.SetDriveMode(GpioPinDriveMode.Output);

                var varSpiCon = new SpiConnectionSettings(0);

                /* Set the SPI buss to 10 Mhz */
                varSpiCon.ChipSelectLine = 10 * (10 ^ 6);

                /* set the clock polarity and phase to: CPOL = 0, CPHA = 0 */
                varSpiCon.Mode = SpiMode.Mode0;

                String spiDeviceSelector = SpiDevice.GetDeviceSelector();
                IReadOnlyList<DeviceInformation> devices = await DeviceInformation.FindAllAsync(spiDeviceSelector);

                _spiDevice = await SpiDevice.FromIdAsync(devices[0].Id, varSpiCon);

            }
            catch(Exception ex) {
                throw new Exception("SPI Initialization Failed", ex);
            }


                                                                                            /* When communicating with a PICC we need a timeout if something goes wrong. */
                                                                                            /* f_timer = 13.56 MHz / (2*TPreScaler+1) where TPreScaler = [TPrescaler_Hi:TPrescaler_Lo]. */
                                                                                            /* TPrescaler_Hi are the four low bits in TModeReg. TPrescaler_Lo is TPrescalerReg. */

            WriteSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.TModeReg, 0x80);          /* TAuto=1; timer starts automatically at the end of the transmission in all communication modes at all speeds */
            WriteSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.TPrescalerReg, 0xA9);     /* TPreScaler = TModeReg[3..0]:TPrescalerReg, ie 0x0A9 = 169 => f_timer=40kHz, ie a timer period of 25ms. */
            WriteSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.TReloadRegH, 0x03);       /* Reload timer with 0x3E8 = 1000, ie 25ms before timeout. */
            WriteSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.TReloadRegL, 0xE8);

            Reset();
        }

        /**
         * FUNCTION NAME : Reset
         * DESCRIPTION   : Reset the RFID reader
         * INPUT         : -
         * OUTPUT        : - 
         * NOTE          :  By Doing the follwing code will make the MRFC make a hardreset, and by doing so it forgets prevoius settings
         *                  //_resetPin.Write(GpioPinValue.Low);
         *                  //System.Threading.Tasks.Task.Delay(50).Wait();
         *                  //_resetPin.Write(GpioPinValue.High);
         *                  //System.Threading.Tasks.Task.Delay(50).Wait();
         */
        protected void Reset() {
            /* Software reset*/
            WriteSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.CommandReg, (Byte)MFRC522Registermap.PCD_Command.PCD_SoftReset);	// Issue the SoftReset command.

            /* Force 100% ASK modulation */
            WriteSpi(WRITE,  (Byte)MFRC522Registermap.PCD_Register.TxASKReg , 0x40);

            /* Set CRC to 0x6363 */
            WriteSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.ModeReg, 0x3D);

            /* Enable antenna */
            WriteSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.TxControlReg, 0x03);

            Debug.WriteLine("Finnished");
        }

        /**
         * FUNCTION NAME : WriteToFifo
         * DESCRIPTION   : 
         * INPUT         : -
         * OUTPUT        : - 
         * NOTE          : -
         */

        protected void WriteToFifo(params byte[] values) {
            foreach (var b in values)
                WriteSpi(WRITE, (byte)MFRC522Registermap.PCD_Register.FIFODataReg, b);
        }

        /**
         * FUNCTION NAME : ReadFromFifo
         * DESCRIPTION   : 
         * INPUT         : -
         * OUTPUT        : - 
         * NOTE          : -
         */
        protected ushort ReadFromFifoShort() {
            var low = WriteSpi(READ, (byte)MFRC522Registermap.PCD_Register.FIFODataReg, 0x00)[1];
            var high = WriteSpi(READ, (byte)MFRC522Registermap.PCD_Register.FIFODataReg, 0x00)[1] << 8;

            return (ushort)(high | low);
        }
        protected void PCD_CalculateCRC(byte[] data,		///< In: Pointer to the data to transfer to the FIFO for CRC calculation.
										byte length,	///< In: The number of bytes to transfer.
										byte[] result) {  ///< Out: Pointer to result buffer. Result is written to result[0..1], low byte first.

            WriteSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.CommandReg,  (Byte)MFRC522Registermap.PCD_Command.PCD_Idle);        // Stop any active command.
            WriteSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.DivIrqReg, 0x04);             // Clear the CRCIRq interrupt request bit
            WriteSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.FIFOLevelReg, 0x80);     // FlushBuffer = 1, FIFO initialization
            WriteRegisters(WRITE, (Byte)MFRC522Registermap.PCD_Register.FIFODataReg, length, data);   // Write data to the FIFO
            WriteSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.CommandReg, (Byte)MFRC522Registermap.PCD_Command.PCD_CalcCRC);     // Start the calculation

            // Wait for the CRC calculation to complete. Each iteration of the while-loop takes 17.73ms.
            UInt16 i = 5000;
            byte n;
            while (true)
            {
                n = PCD_ReadRegister(DivIrqReg);    // DivIrqReg[7..0] bits are: Set2 reserved reserved MfinActIRq reserved CRCIRq reserved reserved
                if (n & 0x04)
                {                       // CRCIRq bit set - calculation done
                    break;
                }
                if (--i == 0)
                {                       // The emergency break. We will eventually terminate on this one after 89ms. Communication with the MFRC522 might be down.
                    return STATUS_TIMEOUT;
                }
            }
            WriteSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.CommandReg, (Byte)MFRC522Registermap.PCD_Command.PCD_Idle);        // Stop calculating CRC for new content in the FIFO.

            // Transfer the result from the registers to the result buffer
            result[0] = ReadRegister(CRCResultRegL);
            result[1] = PCD_ReadRegister(CRCResultRegH);
            return STATUS_OK;
        } // End PCD_CalculateCRC()

        protected void MIFARE_Read(byte blockAddr, 	    /* MIFARE Classic: The block (0-0xff) number. MIFARE Ultralight: The first page to return data from. */
								   byte[] buffer,		    /* The buffer to store the data in */
								   byte[] bufferSize){      /* Buffer size, at least 18 bytes. Also number of bytes returned if STATUS_OK. */

            MFRC522Registermap.StatusCode result;

            // Sanity check
            if (buffer == null || bufferSize[] < 18)
                return /*STATUS_NO_ROOM*/;
            
            // Build command buffer
            buffer[0] = (Byte)MFRC522Registermap.PICC_Command.PICC_CMD_MF_READ;
            buffer[1] = blockAddr;
            // Calculate CRC_A
            result = CalculateCRC(buffer, 2, buffer[2]);
            if (result != MFRC522Registermap.StatusCode.STATUS_OK)
                return /*result*/;

            // Transmit the buffer and receive the response, validate CRC_A.
            return TransceiveData(buffer, 4, buffer, bufferSize, null, 0, true);
        } // End MIFARE_Read()



        /**
        * FUNCTION NAME : MIFARE_Ultralight_Write
        * DESCRIPTION   : -
        * INPUT         : -
        * OUTPUT        : - 
        * NOTE          : 
        */
         protected void MIFARE_Ultralight_Write(byte page, 		    /* The page (2-15) to write to. */
												byte[] buffer,	    /* The 4 bytes to write to the PICC */
												byte bufferSize) {  /* Buffer size, must be at least 4 bytes. Exactly 4 bytes are written. */

            MFRC522Registermap.StatusCode result;

            // Sanity check
            if (buffer == null || bufferSize < 4)
                return /*STATUS_INVALID*/ ;

            // Build commmand buffer
            byte []cmdBuffer = new byte[6];
            cmdBuffer[0] = PICC_CMD_UL_WRITE;
            cmdBuffer[1] = page;
            Array.Copy(cmdBuffer[2], buffer, 4);

            // Perform the write
            result = MIFARE_Transceive(cmdBuffer, 6); // Adds CRC_A and checks that the response is MF_ACK.
            if (result != STATUS_OK)
            {
                return result;
            }
            return STATUS_OK;
        } // End MIFARE_Ultralight_Write()




        /**
        * FUNCTION NAME : Transvice
        * DESCRIPTION   : 
        * INPUT         : -
        * OUTPUT        : -
        * NOTE          : A transceiver is a combination transmitter/receiver in a single package. The term applies to wireless 
        *                 communications devices such as cellular telephones, cordless telephone sets, handheld two-way radios, 
        *                 and mobile two-way radios.
        */

        protected void Transvice(bool isCRCEnable, params byte[] val) {
            if (isCRCEnable) {
                // Enable CRC
                WriteSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.TxModeReg, 0x80);
                WriteSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.RxModeReg, 0x80);
            }

            // Put reader in Idle mode
            WriteSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.CommandReg,  (Byte)MFRC522Registermap.PCD_Command.PCD_Idle);

            // Clear the FIFO
            WriteSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.FIFOLevelReg, 0x80);

            // Write the data to the FIFO
            WriteToFifo(val);

            // Put reader in Transceive mode and start sending
            WriteSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.CommandReg, (Byte)MFRC522Registermap.PCD_Command.PCD_Transceive);
            WriteSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.BitFramingReg, 0x80);

            // Wait for (a generous) 25 ms
            System.Threading.Tasks.Task.Delay(25).Wait();

            // Stop sending
            WriteSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.BitFramingReg, 0x80);

            if (isCRCEnable) {
                // Disable CRC
                WriteSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.TxModeReg, 0x80);
                WriteSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.RxModeReg, 0x80);
            }
        }


        /**
        * FUNCTION NAME : ReadRegisters
        * DESCRIPTION   : Reads many registers bytes
        * INPUT         : -
        * OUTPUT        : -
        * NOTE          : This function is not implemented yet
        */

        protected void ReadRegisters(byte reg,          /* The register to read from. One of the PCD_Register enums. */
                                     byte count,        /* The number of bytes to read */
                                     byte[] values,     /* Byte array to store the values in. */
                                     byte rxAlign) {    /* Only bit positions rxAlign..7 in values[0] are updated. */

            if (count == 0)
                return;

            // MSB == 1 is for reading. LSB is not used in address. Datasheet section 8.1.2.3.
            reg |= 0x80;     
            var writeBuffer = new byte[] { reg, 0x00 };

            
            // Select slave
            _dataCommandPin.Write(GpioPinValue.Low); /* Target slave */
            for(byte idx = 0;  idx < count; idx++) {
                /*if (index == 0 && rxAlign) {      
                    // Only update bit positions rxAlign..7 in values[0]
                    // Create bit mask for bit positions rxAlign..7
                    byte mask = 0x00;
                    for (byte i = rxAlign; i <= 7; i++){
                        mask |= 0x01;
                        mask <<=  i;
                    }
                    // Read value and tell that we want to read the same address again.
                    byte value = WriteSpi(READ, reg, 0x00)[1];
                    // Apply mask to both current value of values[0] and the new data in value.
                    //values[0] = (values[index] & ~mask) | (value & mask);
                } */
            }
            _dataCommandPin.Write(GpioPinValue.High);   /* Release slave again */
        } // End PCD_ReadRegister()


        /**
         * FUNCTION NAME : WriteSpi
         * DESCRIPTION   : This function write and read data over the SPI bus
         * INPUT         : byte[]
         * OUTPUT        : byte[] { 0. Written, 1. Readed } 
         * NOTE          : The MSB of the first byte defines the mode used. To read data from the MFRC522 the 
         *                 MSB is set to logic 1. To write data to the MFRC522 the MSB must be set to logic 0. Bits 6 
         *                 to 1 define the address and the LSB is set to logic 0.
         */
        protected byte[] WriteSpi(String operation, byte register, byte value) {
            
            switch (operation.ToLower()) {
                case READ:
                    register |= 0x80; /* need to or the register in order to make a read operation because MSB must be set 1 for read */
                break;
                case WRITE:
                    /* Leftshifting is done in enum PCD_Register so do nothing, just break and don't let it pass to default */
                break;
                default: /* Undef mode */
                    return null;
            }
            var writeBuffer = new byte[] { register, value };

            var readBuffer = new byte[writeBuffer.Length];

            _spiDevice.TransferFullDuplex(writeBuffer, readBuffer);

            return readBuffer;
        }

        /**
         * FUNCTION NAME : PCD_PerformSelfTest
         * DESCRIPTION   : This function do a selftest to see if its working properly
         * INPUT         : -
         * OUTPUT        : -
         * NOTE          : This function is not implemeted yet
         */

        protected bool PCD_PerformSelfTest() {
            /* This follows directly the steps outlined in 16.1.1 */

            /* 1. Perform a soft reset. */
            Reset();

            /* 2. Clear the internal buffer by writing 25 bytes of 00h */
            byte[] ZEROES = new byte[25];
            foreach(var e in ZEROES)
                ZEROES[e] = e;
           
            // flush the FIFO buffer
            WriteSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.FIFOLevelReg, 0x80);

            // write 25 bytes of 00h to FIFO
            WriteToFifo(ZEROES);

            // transfer to internal buffer
            WriteSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.CommandReg, (Byte)MFRC522Registermap.PCD_Command.PCD_Mem);     

            // 3. Enable self-test
            WriteSpi(WRITE, (Byte) MFRC522Registermap.PCD_Register.AutoTestReg, 0x09);

            // 4. Write 00h to FIFO buffer
            WriteSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.FIFODataReg, 0x00);

            // 5. Start self-test by issuing the CalcCRC command
            WriteSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.CommandReg, (Byte)MFRC522Registermap.PCD_Command.PCD_CalcCRC);

            // 6. Wait for self-test to complete
            byte i;
            byte n;
            for (i = 0; i < 255; i++) {
                n = WriteSpi(READ, (Byte)MFRC522Registermap.PCD_Register.DivIrqReg, 0x00)[1];    // DivIrqReg[7..0] bits are: Set2 reserved reserved MfinActIRq reserved CRCIRq reserved reserved
                /*if (n & 0x04){ 
                    break; // CRCIRq bit set - calculation done
                }*/
            }
            WriteSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.CommandReg, (Byte)MFRC522Registermap.PCD_Command.PCD_Idle);        // Stop calculating CRC for new content in the FIFO.

            // 7. Read out resulting 64 bytes from the FIFO buffer.
            byte[] result = new byte[64];

            ReadRegisters((Byte)MFRC522Registermap.PCD_Register.FIFODataReg, 64, result, 0);

            // Auto self-test done
            // Reset AutoTestReg register to be 0 again. Required for normal operation.
            WriteSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.AutoTestReg, 0x00);

            // Determine firmware version (see section 9.3.4.8 in spec)
            byte version = WriteSpi(READ, (Byte)MFRC522Registermap.PCD_Register.VersionReg, 0x00)[1]; //return of the Byte array is {Write, Read} there for me must chose array 1

            return true;
        } // End PCD_PerformSelfTest()
    }
}