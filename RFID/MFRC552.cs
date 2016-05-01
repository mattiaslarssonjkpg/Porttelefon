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
        private const String READ = "readreg", WRITE = "writereg";

        private GpioController _gpioController;
        private SpiDevice _spiDevice;
        private GpioPin _IRQPin, _resetPin;

        /**
         * FUNCTION NAME : openMFRC522
         * DESCRIPTION   : Open up SPI bus so that we can start sending and reciving data over the bus
         * INPUT         : -
         * OUTPUT        : - 
         * NOTE          : Futher implementaions might be needed
         */

            public async Task openMFRC522() {
            try {
                /* Get defualt Gpiocontroller */
                _gpioController = GpioController.GetDefault();

                /* Initialize a pin as output for the hardware Reset line on the RFID */
                _resetPin = _gpioController.OpenPin(20); //reset pin is now on 21
                _resetPin.Write(GpioPinValue.High);
                _resetPin.SetDriveMode(GpioPinDriveMode.Output);
                
                var varSpiCon = new SpiConnectionSettings(0);

                /* Set the SPI buss to 1 Mhz */
                varSpiCon.ClockFrequency = 1000000;

                /* set the clock polarity and phase to: CPOL = 0, CPHA = 0 */
                varSpiCon.Mode = SpiMode.Mode0;

                String spiDeviceSelector = SpiDevice.GetDeviceSelector();
                IReadOnlyList<DeviceInformation> devices = await DeviceInformation.FindAllAsync(spiDeviceSelector);

                _spiDevice = await SpiDevice.FromIdAsync(devices[0].Id, varSpiCon);

            }
            catch(Exception ex) {
                throw new Exception("SPI Initialization Failed", ex);
            }

            TransferToSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.TModeReg, 0x80);      /* When communicating with a PICC we need a timeout if something goes wrong. */
            TransferToSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.TPrescalerReg, 0xA9); /* f_timer = 13.56 MHz / (2*TPreScaler+1) where TPreScaler = [TPrescaler_Hi:TPrescaler_Lo]. */
            TransferToSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.TReloadRegH, 0x03);   /* TPrescaler_Hi are the four low bits in TModeReg. TPrescaler_Lo is TPrescalerReg. */
            TransferToSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.TReloadRegL, 0xE8);   /* TAuto=1; timer starts automatically at the end of the transmission in all communication modes at all speeds */
            /* TPreScaler = TModeReg[3..0]:TPrescalerReg, ie 0x0A9 = 169 => f_timer=40kHz, ie a timer period of 25ms. */
            /* Reload timer with 0x3E8 = 1000, ie 25ms before timeout. */
            Reset();
            
        }

        /**
        * FUNCTION NAME : TransferToSpi
        * DESCRIPTION   : This function write and read data over the SPI bus
        * INPUT         : byte[]
        * OUTPUT        : byte[] { 0. Written, 1. Readed } 
        * NOTE          : The MSB of the first byte defines the mode used. To read data from the MFRC522 the 
        *                 MSB is set to logic 1. To write data to the MFRC522 the MSB must be set to logic 0. Bits 6 
        *                 to 1 define the address and the LSB is set to logic 0.
        */
        public byte[] TransferToSpi(String operation, byte register, byte value){
            switch (operation.ToLower()) {
                case WRITE:
                    register <<= 1;
                break;
                case READ:
                    register <<= 1;
                    register = (Byte)(0x80 | register);
                    break;
                default:
                    return null;
            }
            byte[] writeBuffer = { register, value };
            byte[] readBuffer = new byte[2];
            _spiDevice.TransferFullDuplex(writeBuffer, readBuffer);

            return readBuffer;
        }


        /**
         * FUNCTION NAME : ReadSpi
         * DESCRIPTION   : Reading registers of the PIC
         * INPUT         : string, byte
         * OUTPUT        : byte[]
         * NOTE          : String operation checks if the operation is made if not, we return null 
         */

        public byte[] ReadSpi(String operation, byte register)
        {
            switch (operation.ToLower()) {
                case READ:
                    register <<= 1;
                    register = (Byte)(0x80 | register);
                    break;
                default:
                    return null;
            }
            byte[] readRegister = { register, 0x00 };
            _spiDevice.Read(readRegister);
            return readRegister;
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

        public void Reset() {

            _resetPin.Write(GpioPinValue.Low);
            System.Threading.Tasks.Task.Delay(50).Wait();
            _resetPin.Write(GpioPinValue.High);
            System.Threading.Tasks.Task.Delay(50).Wait();

            /* Software reset*/
            TransferToSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.CommandReg, (Byte)MFRC522Registermap.PCD_Command.PCD_SoftReset);	// Issue the SoftReset command.

            /* Force 100% ASK modulation */
            TransferToSpi(WRITE,  (Byte)MFRC522Registermap.PCD_Register.TxASKReg , 0x40);

            /* Set CRC to 0x6363 */
            TransferToSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.ModeReg, 0x3D);

            /* Enable antenna */
            TransferToSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.TxControlReg, 0x03);

            Debug.WriteLine("Finnished");
        }

        /**
         * FUNCTION NAME : WriteToFifo
         * DESCRIPTION   : 
         * INPUT         : -
         * OUTPUT        : - 
         * NOTE          : -
         */

        public void WriteToFifo(params byte[] values) {
            foreach (var b in values)
                TransferToSpi(WRITE, (byte)MFRC522Registermap.PCD_Register.FIFODataReg, b);
        }

        /**
         * FUNCTION NAME : ReadFromFifo
         * DESCRIPTION   : 
         * INPUT         : -
         * OUTPUT        : - 
         * NOTE          : -
         */
        //public ushort ReadFromFifoShort() {
        //var low = TransferToSpi(READ, (byte)MFRC522Registermap.PCD_Register.FIFODataReg, 0x00)[1];
        //var high = TransferToSpi(READ, (byte)MFRC522Registermap.PCD_Register.FIFODataReg, 0x00)[1] << 8;

        //return (ushort)(high | low);
        //}

        /**
         * FUNCTION NAME : CalculateCRC
         * DESCRIPTION   : 
         * INPUT         : -
         * OUTPUT        : - 
         * NOTE          : 
         */
        public void CalculateCRC(byte[] Indata) {  

            TransferToSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.CommandReg,  (Byte)MFRC522Registermap.PCD_Command.PCD_Idle);        // Stop any active command.

            TransferToSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.DivIrqReg, 0x04);             // Clear the CRCIRq interrupt request bit
            TransferToSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.FIFOLevelReg, 0x80);     // FlushBuffer = 1, FIFO initialization

            //TransferToSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.FIFODataReg, data);   // Write data to the FIFO
            TransferToSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.CommandReg, (Byte)MFRC522Registermap.PCD_Command.PCD_CalcCRC);     // Start the calculation

            // Wait for the CRC calculation to complete. Each iteration of the while-loop takes 17.73ms.
            UInt16 i = 5000;
            byte n;
            /*while (true)
            {
                n = ReadRegister(DivIrqReg);    // DivIrqReg[7..0] bits are: Set2 reserved reserved MfinActIRq reserved CRCIRq reserved reserved
                if (n & 0x04)
                {                       // CRCIRq bit set - calculation done
                    break;
                }
                if (--i == 0)
                {                       // The emergency break. We will eventually terminate on this one after 89ms. Communication with the MFRC522 might be down.
                    return STATUS_TIMEOUT;
                }
            }
            TransferToSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.CommandReg, (Byte)MFRC522Registermap.PCD_Command.PCD_Idle);        // Stop calculating CRC for new content in the FIFO.

            // Transfer the result from the registers to the result buffer
            result[0] = ReadRegister(CRCResultRegL);
            result[1] = PCD_ReadRegister(CRCResultRegH);
            return STATUS_OK;*/
        } // End PCD_CalculateCRC()


        /**
         * FUNCTION NAME : CalculateCRC
         * DESCRIPTION   : 
         * INPUT         : -
         * OUTPUT        : - 
         * NOTE          : 
         */

        /*protected void MIFARE_Read(byte blockAddr, 	    /* MIFARE Classic: The block (0-0xff) number. MIFARE Ultralight: The first page to return data from. /
								   byte[] buffer,		    /* The buffer to store the data in /
								   byte[] bufferSize){      /* Buffer size, at least 18 bytes. Also number of bytes returned if STATUS_OK. /

            MFRC522Registermap.StatusCode result;

            // Sanity check
            if (buffer == null || bufferSize[] < 18)
                return /*STATUS_NO_ROOM/;
            
            // Build command buffer
            buffer[0] = (Byte)MFRC522Registermap.PICC_Command.PICC_CMD_MF_READ;
            buffer[1] = blockAddr;
            // Calculate CRC_A
            result = CalculateCRC(buffer, 2, buffer[2]);
            if (result != MFRC522Registermap.StatusCode.STATUS_OK)
                return /*result/;

            // Transmit the buffer and receive the response, validate CRC_A.
            return TransceiveData(buffer, 4, buffer, bufferSize, null, 0, true);
        } // End MIFARE_Read()*/




        /**
        * FUNCTION NAME : MIFARE_Ultralight_Write
        * DESCRIPTION   : -
        * INPUT         : -
        * OUTPUT        : - 
        * NOTE          : 
        */
        public void MIFARE_Ultralight_Write(byte page, 		    /* The page (2-15) to write to. */
												byte[] buffer,	    /* The 4 bytes to write to the PICC */
												byte bufferSize) {  /* Buffer size, must be at least 4 bytes. Exactly 4 bytes are written. */

            MFRC522Registermap.StatusCode result;

            // Sanity check
            if (buffer == null || bufferSize < 4)
                return /*STATUS_INVALID*/ ;

            // Build commmand buffer
            byte []cmdBuffer = new byte[6];
            //cmdBuffer[0] = PICC_CMD_UL_WRITE;
            cmdBuffer[1] = page;
            //Array.Copy(cmdBuffer[2], buffer, 4);

            // Perform the write
            /*result = MIFARE_Transceive(cmdBuffer, 6); // Adds CRC_A and checks that the response is MF_ACK.
            if (result != STATUS_OK)
            {
                return result;
            }
            return STATUS_OK;*/
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

        public void Transvice(bool isCRCEnable, params byte[] val) {
            if (isCRCEnable) {
                // Enable CRC
                TransferToSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.TxModeReg, 0x80);
                TransferToSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.RxModeReg, 0x80);
            }

            // Put reader in Idle mode
            TransferToSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.CommandReg,  (Byte)MFRC522Registermap.PCD_Command.PCD_Idle);

            // Clear the FIFO
            TransferToSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.FIFOLevelReg, 0x80);

            // Write the data to the FIFO
            WriteToFifo(val);

            // Put reader in Transceive mode and start sending
            TransferToSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.CommandReg, (Byte)MFRC522Registermap.PCD_Command.PCD_Transceive);
            TransferToSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.BitFramingReg, 0x80);

            // Wait for (a generous) 25 ms
            System.Threading.Tasks.Task.Delay(25).Wait();

            // Stop sending
            TransferToSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.BitFramingReg, 0x80);

            if (isCRCEnable) {
                // Disable CRC
                TransferToSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.TxModeReg, 0x80);
                TransferToSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.RxModeReg, 0x80);
            }
        }
    }
}