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
using static IoTCore.RFID_Constans.MFRC522Registermap;

namespace IoTCore {
    class MFRC522 {
        private const Byte DATA_COMMAND_PIN = 0; /* We use GPIO 22 since it's conveniently near the SPI pins */
        private const String READ = "readreg", WRITE = "writereg";

        private GpioController _gpioController;
        private SpiDevice _spiDevice;
        private GpioPin _resetPin;
        public const Byte MI_OK = 0;
        public const Byte MI_NOTAGERR = 1;
        public const Byte MI_ERR = 2;
        public const Byte MAX_LEN = 16;
        #region InitFunctions
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
                
                var varSpiCon = new SpiConnectionSettings(DATA_COMMAND_PIN);

                varSpiCon.ChipSelectLine = DATA_COMMAND_PIN;
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

        public void Reset()
        {

            _resetPin.Write(GpioPinValue.Low);
            System.Threading.Tasks.Task.Delay(50).Wait();
            _resetPin.Write(GpioPinValue.High);
            System.Threading.Tasks.Task.Delay(50).Wait();

            /* Software reset*/
            TransferToSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.CommandReg, (Byte)MFRC522Registermap.PCD_Command.PCD_SoftReset);	// Issue the SoftReset command.

            /* Force 100% ASK modulation */
            TransferToSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.TxASKReg, 0x40);

            /* Set CRC to 0x6363 */
            TransferToSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.ModeReg, 0x3D);

            /* Enable antenna */
            TransferToSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.TxControlReg, 0x03);

            Debug.WriteLine("Finnished");
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
         * Sets the bits given in mask in register reg.
         */
        private void PCD_SetRegisterBitMask(byte reg,  ///< The register to update. One of the PCD_Register enums.
                                        byte mask   ///< The bits to set.
                                    ){
            byte tmp;
            tmp = TransferToSpi(READ, reg, 0x00)[1];
            TransferToSpi(WRITE, reg, (Byte)(tmp | mask));         // set bit mask
        } // End PCD_SetRegisterBitMask()

        /**
         * Clears the bits given in mask from register reg.
         */
        private void PCD_ClearRegisterBitMask(byte reg,    ///< The register to update. One of the PCD_Register enums.
                                              byte mask   ///< The bits to clear.
                                      ){
            byte tmp;
            tmp = TransferToSpi(READ, reg, 0x00)[1];
            TransferToSpi(WRITE, reg, ((Byte)(tmp & (~mask))));      // clear bit mask
        } // End PCD_ClearRegisterBitMask()




        #endregion

        #region CalculateCRC
        /**
         * FUNCTION NAME : CalulateCRC
         * DESCRIPTION   : 
         * INPUT         : -
         * OUTPUT        : - 
         * NOTE          : 
         */

        public List<Byte> CalulateCRC(List<Byte> Indata)
        {

            Byte[] result = new Byte[2];
            TransferToSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.CommandReg, (Byte)MFRC522Registermap.PCD_Command.PCD_Idle);        /* Stop any active command. */

            TransferToSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.DivIrqReg, 0x04);     /* Clear the CRCIRq interrupt request bit */
            TransferToSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.FIFOLevelReg, 0x80);  /* FlushBuffer = 1, FIFO initialization */

            foreach (var b in Indata){
                TransferToSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.FIFODataReg, b);   /* Write data to the FIFO */
            }
            TransferToSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.CommandReg, (Byte)MFRC522Registermap.PCD_Command.PCD_CalcCRC);
            Indata.Clear();
            while (true)
            {
                byte r = TransferToSpi(READ, (Byte)MFRC522Registermap.PCD_Register.DivIrqReg, 0x00)[0];
                
                if ((r & 0x04) != 0) { 
                    break;
                }
            }
            
            TransferToSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.CommandReg, (Byte)MFRC522Registermap.PCD_Command.PCD_Idle);        /* Stop calculating CRC for new content in the FIFO. */

            Indata.Add(TransferToSpi(READ, (Byte)MFRC522Registermap.PCD_Register.CRCResultRegL, 0x00)[0]);
            Indata.Add(TransferToSpi(READ, (Byte)MFRC522Registermap.PCD_Register.CRCResultRegH, 0x00)[0]);

            System.Threading.Tasks.Task.Delay(20).Wait(); /* Wait for (a generous) 25 ms */

            return Indata;
        }
        #endregion

        #region MIFARE_Read
        /**
         * FUNCTION NAME : CalulateCRC
         * DESCRIPTION   : 
         * INPUT         : -
         * OUTPUT        : - 
         * NOTE          : 
         */


        public void MIFARE_Read(byte blockAddr, 	    /* MIFARE Classic: The block (0-0xff) number. MIFARE Ultralight: The first page to return data from. */
								   List<byte> buffer,	/* The buffer to store the data in */
								   byte[] bufferSize){  /* Buffer size, at least 18 bytes. Also number of bytes returned if STATUS_OK. */

            List<Byte> recvData = new List<byte> { };
            recvData.Add((Byte) MFRC522Registermap.PICC_Command.PICC_CMD_MF_READ);
            recvData.Add(blockAddr);
            var result = CalulateCRC(buffer);
            recvData.Clear();
            recvData.Add(result[0]);
            recvData.Add(result[1]);

            var r = MFRC522_ToCard( (Byte)MFRC522Registermap.PCD_Command.PCD_Transceive, recvData); // [0] = status, [1] = backData, [2] = backLen
            if ((r[0] != MI_OK)) //where its zero write status
                Debug.WriteLine("Error while reading!");

           Debug.WriteLine("Got data size: " + r[2]);
           if (r[1] == 16)
               Debug.WriteLine("Sector " + blockAddr + " " + result[1]);

            //return backData
        }
        #endregion

        #region MFRC522_Request
        public List<Byte> MFRC522_Request(byte reqMode) {
          
            List<Byte> TagType = new List<byte> { };
            
            TransferToSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.BitFramingReg, 0x07);
    
            TagType.Add(reqMode);
            var status = MFRC522_ToCard((Byte)MFRC522Registermap.PCD_Command.PCD_Transceive, TagType);
  
            if ((status[0] != MI_OK) | (status[2] != 0x10))
                status[0] = MI_ERR;

            
            return status;
        }
        #endregion

        #region Ultralight Write
        /**
        * FUNCTION NAME : MIFARE_Ultralight_Write
        * DESCRIPTION   : -
        * INPUT         : -
        * OUTPUT        : - 
        * NOTE          : Note done yet... Need more work
        */
        public void MIFARE_Ultralight_Write(Byte blockAddr, 		    /* The page (2-15) to write to. */
                                            List<Byte> writeData) {  /* Buffer size, must be at least 4 bytes. Exactly 4 bytes are written. */

            List<Byte> buff = new List<Byte> { };
            List<Byte> crc = new List<Byte> { };
            buff.Add((Byte)MFRC522Registermap.PICC_Command.PICC_CMD_MF_WRITE);
            buff.Add(blockAddr);
            crc = CalulateCRC(buff);
            buff.Clear();
            buff.Add(crc[0]);
            buff.Add(crc[1]);
            crc.Clear();

            var status = MFRC522_ToCard((Byte)MFRC522Registermap.PCD_Command.PCD_Transceive, buff); // [0] = status, [1] = backData, [2] = backLen
            if ((status[0] != MI_OK) || (status[2] != 4) || (status[1] & 0x0F) == 0x0A)
                status[0] = MI_ERR;

            Debug.WriteLine(status[2] + " backdata &0x0F == 0x0A " + (status[1] & 0x0F));
            if (status[0] == MI_OK) { 
                foreach (var b in writeData)
                    writeData[b] = writeData[b];
                List<Byte> buf = new List<Byte> { };
                crc = CalulateCRC(writeData);
                buf.Add(crc[0]);
                buf.Add(crc[1]);
                status = MFRC522_ToCard((Byte)MFRC522Registermap.PCD_Command.PCD_Transceive, buf); // [0] = status, [1] = backData, [2] = backLen
                if ((status[0] != MI_OK) || (status[2] != 4) || ((status[1] & 0x0F) != 0x0A)) 
                  Debug.WriteLine("Error while writing");
                if (status[0] == MI_OK)
                    Debug.WriteLine("Data writen");
                else
                    Debug.WriteLine("Something failed");
            }
        }
        #endregion

        #region TO_CARD
        private List<Byte> MFRC522_ToCard(Byte command, List<Byte> sendData) {

            Byte[] backData = new Byte[6];
            byte backLen = 0;
            List<byte[]> bLen = new List<byte[]>();
            var status = MI_ERR;
            byte irqEn = 0x00;
            byte waitIRq = 0x00;
            byte lastBits = 0x00;
            
            if (command == (Byte)MFRC522Registermap.PCD_Command.PCD_MFAuthent){
                irqEn = 0x12;
                waitIRq = 0x10;
            }
            if (command == (Byte)MFRC522Registermap.PCD_Command.PCD_Transceive) {
                irqEn = 0x77;
                waitIRq = 0x30;
            }

            TransferToSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.ComIEnReg, (Byte)(irqEn | 0x80));
            TransferToSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.ComIrqReg, 0x80);
            TransferToSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.FIFODataReg, 0x80);
    
            TransferToSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.CommandReg, (Byte)MFRC522Registermap.PCD_Command.PCD_Idle);

            foreach (var bSend in sendData)
                TransferToSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.FIFODataReg, bSend);

            TransferToSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.CommandReg, command);

            if (command == (Byte)MFRC522Registermap.PCD_Command.PCD_Transceive)
                TransferToSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.BitFramingReg, 0x80);
            byte n = 0, i = 0;
            
            while (true) {
                n = TransferToSpi(READ, (Byte)MFRC522Registermap.PCD_Register.ComIrqReg, 0x00)[0];
                i = (Byte)(i - 1);
                if ((i != 0) && ((n & 0x01) != 0) && ((n&waitIRq) != 0))
                    break;
            }
            TransferToSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.BitFramingReg, 0x80);


            if (i != 0) {
                if ((TransferToSpi(READ, (Byte)MFRC522Registermap.PCD_Register.ErrorReg, 0x00)[0] & 0x1B) == 0x00)
                    status = MI_OK;

                if ((n & irqEn & 0x01) != 0)
                    status = MI_NOTAGERR;

                if (command ==  (Byte)MFRC522Registermap.PCD_Command.PCD_Transceive) {
                    n = TransferToSpi(READ, (Byte)MFRC522Registermap.PCD_Register.FIFOLevelReg, 0x00)[0];
                    lastBits = (Byte)(TransferToSpi(READ, (Byte)MFRC522Registermap.PCD_Register.ControlReg, 0x00)[0] & 0x07);
                    if (lastBits != 0)
                        backLen = (Byte)((n - 1) * 8 + lastBits);
                    else
                        backLen = (Byte)(n * 8);

                    if (n == 0)
                        n = 1;

                    if (n > MAX_LEN)
                        n = MAX_LEN;

                    i = 0;
                    while (i < n) {
                        backData[i]  = TransferToSpi(READ, (Byte)MFRC522Registermap.PCD_Register.FIFODataReg,  0x00)[1];
                        i = (Byte)(i + 1);
                    }
                }
                else
                    status = MI_ERR;
                }
            
            List<Byte> arr = new List<Byte> { };
            arr.Add(status);
            arr.Add(backLen);
            foreach (var a in backData)
                arr.Add(a);
            
            return arr;
        }
        #endregion

        #region IsNewCard
        /**
         * Returns true if a PICC responds to PICC_CMD_REQA.
         * Only "new" cards in state IDLE are invited. Sleeping cards in state HALT are ignored.
         * 
         * @return bool
         */
        public bool PICC_IsNewCardPresent()
        {
            byte[] bufferATQA = new byte[2];
            byte[] bufferSize = new byte[(Byte)bufferATQA.Length];

            byte result = (byte)(PICC_RequestA(bufferATQA, bufferSize));
            return (result == (Byte)MFRC522Registermap.StatusCode.STATUS_OK || result == (Byte)MFRC522Registermap.StatusCode.STATUS_COLLISION);
        } // End PICC_IsNewCardPresent()
        #endregion

        #region ReadCardSerial
        /**
         * Simple wrapper around PICC_Select.
         * Returns true if a UID could be read.
         * Remember to call PICC_IsNewCardPresent(), PICC_RequestA() or PICC_WakeupA() first.
         * The read UID is available in the class variable uid.
         * 
         * @return bool
         */
        public bool PICC_ReadCardSerial()
        {
            MFRC522Registermap.StatusCode result = PICC_Select(&uid);
            return (result == (Byte)MFRC522Registermap.StatusCode.STATUS_OK);
        } // End 
        #endregion

        #region PICC_RequestA
        private byte PICC_RequestA(byte[] bufferATQA,	///< The buffer to store the ATQA (Answer to request) in
								   byte[] bufferSize	///< Buffer size, at least two bytes. Also number of bytes returned if STATUS_OK.
										){
            return PICC_REQA_or_WUPA((Byte)MFRC522Registermap.PICC_Command.PICC_CMD_REQA, bufferATQA, bufferSize);
        } // End PICC_RequestA()

        #endregion

        private byte PICC_REQA_or_WUPA(byte command, 		///< The command to send - PICC_CMD_REQA or PICC_CMD_WUPA
									   byte[] bufferATQA,	///< The buffer to store the ATQA (Answer to request) in
									   byte[] bufferSize	///< Buffer size, at least two bytes. Also number of bytes returned if STATUS_OK.
											)
        {
            byte validBits;
            MFRC522Registermap.StatusCode status;

            if (bufferATQA == null || bufferSize.Length < 2)
            {   // The ATQA response is 2 bytes long.
                return (Byte)MFRC522Registermap.StatusCode.STATUS_NO_ROOM;
            }
            
            PCD_ClearRegisterBitMask((Byte)MFRC522Registermap.PCD_Register.CollReg, 0x80);        // ValuesAfterColl=1 => Bits received after collision are cleared.
            validBits = 7;                                  // For REQA and WUPA we need the short frame format - transmit only 7 bits of the last (and only) byte. TxLastBits = BitFramingReg[2..0]
            status = PCD_TransceiveData(command, 1, bufferATQA, bufferSize, validBits,  false);
            if (status != (Byte)MFRC522Registermap.StatusCode.STATUS_OK)
            {
                return (Byte)status;
            }
            if (bufferSize.Length != 2 || validBits != 0)
            {       // ATQA must be exactly 16 bits.
                return (Byte)MFRC522Registermap.StatusCode.STATUS_ERROR;
            }
            return (Byte)MFRC522Registermap.StatusCode.STATUS_OK;
        } // End PICC_REQA_or_WUPA()


        private byte PCD_TransceiveData(byte[] sendData,	///< Pointer to the data to transfer to the FIFO.
										byte[] backData,	///< NULL or pointer to buffer if data should be read back after executing the command.
										byte[] backLen,		///< In: Max number of bytes to write to *backData. Out: The number of bytes returned.
										byte[] validBits,	///< In/Out: The number of valid bits in the last byte. 0 for 8 valid bits. Default NULL.
										byte rxAlign,		///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
										bool checkCRC		///< In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.
								 )
        {
            byte waitIRq = 0x30;        // RxIRq and IdleIRq
            return PCD_CommunicateWithPICC(
                (Byte)MFRC522Registermap.PCD_Command.PCD_Transceive, waitIRq, sendData, 
                backData, backLen, validBits, rxAlign, checkCRC);
        } // End PCD_TransceiveData()

        /**
         * Transfers data to the MFRC522 FIFO, executes a command, waits for completion and transfers data back from the FIFO.
         * CRC validation can only be done if backData and backLen are specified.
         *
         * @return STATUS_OK on success, STATUS_??? otherwise.
         */
        private byte PCD_CommunicateWithPICC(byte command,      ///< The command to execute. One of the PCD_Command enums.
                                             byte waitIRq,       ///< The bits in the ComIrqReg register that signals successful completion of the command.
                                             byte[] sendData,     ///< Pointer to the data to transfer to the FIFO
                                             byte[] backData,     ///< NULL or pointer to buffer if data should be read back after executing the command.
                                             byte[] backLen,      ///< In: Max number of bytes to write to *backData. Out: The number of bytes returned.
                                             byte[] validBits,    ///< In/Out: The number of valid bits in the last byte. 0 for 8 valid bits.
                                             byte rxAlign,       ///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
                                             bool checkCRC       ///< In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.
                                     )
        {
            byte n, _validBits;
            UInt16 i;

            // Prepare values for BitFramingReg
            byte txLastBits = validBits ? validBits : 0;
            byte bitFraming = (Byte)((rxAlign << 4) + txLastBits);      // RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]

            TransferToSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.CommandReg, (Byte)MFRC522Registermap.PCD_Command.PCD_Idle);            // Stop any active command.
            TransferToSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.ComIrqReg, 0x7F);                 // Clear all seven interrupt request bits
            PCD_SetRegisterBitMask((Byte)MFRC522Registermap.PCD_Register.FIFOLevelReg, 0x80);         // FlushBuffer = 1, FIFO initialization
            foreach(var b in sendData)
                TransferToSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.FIFODataReg, b);  // Write sendData to the FIFO
            TransferToSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.BitFramingReg, bitFraming);       // Bit adjustments
            TransferToSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.CommandReg, command);             // Execute the command
            if (command == (Byte)MFRC522Registermap.PCD_Command.PCD_Transceive)
            {
                PCD_SetRegisterBitMask((Byte)MFRC522Registermap.PCD_Register.BitFramingReg, 0x80);    // StartSend=1, transmission of data starts
            }

            // Wait for the command to complete.
            // In PCD_Init() we set the TAuto flag in TModeReg. This means the timer automatically starts when the PCD stops transmitting.
            // Each iteration of the do-while-loop takes 17.86�s.
            i = 2000;
            while (true)
            {
                n = TransferToSpi(READ, (Byte)MFRC522Registermap.PCD_Register.ComIrqReg, 0x00)[1];    // ComIrqReg[7..0] bits are: Set1 TxIRq RxIRq IdleIRq HiAlertIRq LoAlertIRq ErrIRq TimerIRq
                if ((n & waitIRq) != 0)
                // One of the interrupts that signal success has been set.
                    break;
                
            }

            // Stop now if any errors except collisions were detected.
            byte errorRegValue = TransferToSpi(READ, (Byte)MFRC522Registermap.PCD_Register.ErrorReg, 0x00)[1]; // ErrorReg[7..0] bits are: WrErr TempErr reserved BufferOvfl CollErr CRCErr ParityErr ProtocolErr
            if ((errorRegValue & 0x13) != 0 ) //
            {    // BufferOvfl ParityErr ProtocolErr
                return STATUS_ERROR;
            }

            // If the caller wants data back, get it from the MFRC522.
            if (backData && backLen)
            {
                n = TransferToSpi(READ, (Byte)MFRC522Registermap.PCD_Register.FIFOLevelReg, 0x00)[1];         // Number of bytes in the FIFO
                if (n > *backLen)
                {
                    return STATUS_NO_ROOM;
                }
                *backLen = n;                                           // Number of bytes returned
                //TransferToSpi(READ, (Byte)MFRC522Registermap.PCD_Register.FIFODataReg, n, backData, rxAlign);    // Get received data from FIFO
                _validBits = (byte)(TransferToSpi(READ, (Byte)MFRC522Registermap.PCD_Register.ControlReg, 0x00)[1] & 0x07);       // RxLastBits[2:0] indicates the number of valid bits in the last received byte. If this value is 000b, the whole byte is valid.
                if (validBits)
                {
                    *validBits = _validBits;
                }
            }

            // Tell about collisions
            if (errorRegValue & 0x08)
            {       // CollErr
                return STATUS_COLLISION;
            }

            // Perform CRC_A validation if requested.
            if (backData && backLen && checkCRC)
            {
                // In this case a MIFARE Classic NAK is not OK.
                if (*backLen == 1 && _validBits == 4)
                {
                    return STATUS_MIFARE_NACK;
                }
                // We need at least the CRC_A value and all 8 bits of the last byte must be received.
                if (*backLen < 2 || _validBits != 0)
                {
                    return STATUS_CRC_WRONG;
                }
                // Verify CRC_A - do our own calculation and store the control in controlBuffer.
                byte[] controlBuffer = new byte[2];
                MFRC522::StatusCode status = PCD_CalculateCRC(&backData[0], *backLen - 2, &controlBuffer[0]);
                if (status != STATUS_OK)
                {
                    return status;
                }
                if ((backData[*backLen - 2] != controlBuffer[0]) || (backData[*backLen - 1] != controlBuffer[1]))
                {
                    return STATUS_CRC_WRONG;
                }
            }

            return STATUS_OK;
        } // End PCD_CommunicateWithPICC()




        #region GetPICCType

        public byte PICC_GetType(byte sak		///< The SAK byte returned from PICC_Select().
										){
            // 3.2 Coding of Select Acknowledge (SAK)
            // ignore 8-bit (iso14443 starts with LSBit = bit 1)
            // fixes wrong type for manufacturer Infineon (http://nfc-tools.org/index.php?title=ISO14443A)
            sak &= 0x7F;
            switch (sak)
            {
                case 0x04: return (Byte)MFRC522Registermap.PICC_Type.PICC_TYPE_NOT_COMPLETE;   // UID not complete
                case 0x09: return (Byte)MFRC522Registermap.PICC_Type.PICC_TYPE_MIFARE_MINI;
                case 0x08: return (Byte)MFRC522Registermap.PICC_Type.PICC_TYPE_MIFARE_1K;
                case 0x18: return (Byte)MFRC522Registermap.PICC_Type.PICC_TYPE_MIFARE_4K;
                case 0x00: return (Byte)MFRC522Registermap.PICC_Type.PICC_TYPE_MIFARE_UL;
                case 0x10:
                case 0x11: return (Byte)MFRC522Registermap.PICC_Type.PICC_TYPE_MIFARE_PLUS;
                case 0x01: return (Byte)MFRC522Registermap.PICC_Type.PICC_TYPE_TNP3XXX;
                case 0x20: return (Byte)MFRC522Registermap.PICC_Type.PICC_TYPE_ISO_14443_4;
                case 0x40: return (Byte)MFRC522Registermap.PICC_Type.PICC_TYPE_ISO_18092;
                default: return (Byte)MFRC522Registermap.PICC_Type.PICC_TYPE_UNKNOWN;
            }
        } // End PICC_GetType()
        #endregion

        #region PICC_Select
        public byte PICC_Select(Uid[] uid,			///< Pointer to Uid struct. Normally output, but can also be used to supply a known UID.
								byte validBits){    ///< The number of known UID bits supplied in *uid. Normally 0. If set you must also supply uid->size.

            bool uidComplete;
            bool selectDone;
            bool useCascadeTag;
            byte cascadeLevel = 1;
            //MFRC522::StatusCode result;
            byte count;
            byte index;
            byte uidIndex;                  // The first index in uid->uidByte[] that is used in the current Cascade Level.
            Int16 currentLevelKnownBits;    // The number of known UID bits in the current Cascade Level.
            byte[] buffer = new byte[7];    // The SELECT/ANTICOLLISION commands uses a 7 byte standard frame + 2 bytes CRC_A
            byte bufferUsed;                // The number of bytes used in the buffer, ie the number of bytes to transfer to the FIFO.
            byte rxAlign;                   // Used in BitFramingReg. Defines the bit position for the first bit received.
            byte txLastBits;                // Used in BitFramingReg. The number of valid bits in the last transmitted byte. 
            byte[] responseBuffer;
            byte responseLength;

            // Description of buffer structure:
            //		Byte 0: SEL 				Indicates the Cascade Level: PICC_CMD_SEL_CL1, PICC_CMD_SEL_CL2 or PICC_CMD_SEL_CL3
            //		Byte 1: NVB					Number of Valid Bits (in complete command, not just the UID): High nibble: complete bytes, Low nibble: Extra bits. 
            //		Byte 2: UID-data or CT		See explanation below. CT means Cascade Tag.
            //		Byte 3: UID-data
            //		Byte 4: UID-data
            //		Byte 5: UID-data
            //		Byte 6: BCC					Block Check Character - XOR of bytes 2-5
            //		Byte 7: CRC_A
            //		Byte 8: CRC_A
            // The BCC and CRC_A are only transmitted if we know all the UID bits of the current Cascade Level.
            //
            // Description of bytes 2-5: (Section 6.5.4 of the ISO/IEC 14443-3 draft: UID contents and cascade levels)
            //		UID size	Cascade level	Byte2	Byte3	Byte4	Byte5
            //		========	=============	=====	=====	=====	=====
            //		 4 bytes		1			uid0	uid1	uid2	uid3
            //		 7 bytes		1			CT		uid0	uid1	uid2
            //						2			uid3	uid4	uid5	uid6
            //		10 bytes		1			CT		uid0	uid1	uid2
            //						2			CT		uid3	uid4	uid5
            //						3			uid6	uid7	uid8	uid9

            // Sanity checks
            if (validBits > 80)
            {
                return (Byte)MFRC522Registermap.StatusCode.STATUS_INVALID;
            }

            // Prepare MFRC522
            PCD_ClearRegisterBitMask((Byte)MFRC522Registermap.PCD_Register.CollReg, 0x80);        // ValuesAfterColl=1 => Bits received after collision are cleared.

            // Repeat Cascade Level loop until we have a complete UID.
            uidComplete = false;
            while (!uidComplete) {
                // Set the Cascade Level in the SEL byte, find out if we need to use the Cascade Tag in byte 2.
                switch (cascadeLevel)
                {
                    case 1:
                        buffer[0] = (Byte)MFRC522Registermap.PICC_Command.PICC_CMD_SEL_CL1;
                        uidIndex = 0; 
                        useCascadeTag = validBits && uid->size > 4; // When we know that the UID has more than 4 bytes
                        break;

                    case 2:
                        buffer[0] = (Byte)MFRC522Registermap.PICC_Command.PICC_CMD_SEL_CL2;
                        uidIndex = 3;
                        useCascadeTag = validBits && uid->size > 7; // When we know that the UID has more than 7 bytes
                        break;

                    case 3:
                        buffer[0] = (Byte)MFRC522Registermap.PICC_Command.PICC_CMD_SEL_CL3;
                        uidIndex = 6;
                        useCascadeTag = false;                      // Never used in CL3.
                        break;

                    default:
                        return (Byte)MFRC522Registermap.StatusCode.STATUS_INTERNAL_ERROR;
                }

                // How many UID bits are known in this Cascade Level?
                currentLevelKnownBits = validBits - (8 * uidIndex);
                if (currentLevelKnownBits < 0)
                {
                    currentLevelKnownBits = 0;
                }
                // Copy the known bits from uid->uidByte[] to buffer[]
                index = 2; // destination index in buffer[]
                if (useCascadeTag)
                {
                    buffer[index++] = (Byte)MFRC522Registermap.PICC_Command.PICC_CMD_CT;
                }
                byte bytesToCopy = currentLevelKnownBits / 8 + (currentLevelKnownBits % 8 ? 1 : 0); // The number of bytes needed to represent the known bits for this level.
                if (bytesToCopy > 0)
                {
                    byte maxBytes = useCascadeTag ? 3 : 4; // Max 4 bytes in each Cascade Level. Only 3 left if we use the Cascade Tag
                    if (bytesToCopy > maxBytes)
                    {
                        bytesToCopy = maxBytes;
                    }
                    for (count = 0; count < bytesToCopy; count++)
                    {
                        buffer[index++] = uid->uidByte[uidIndex + count];
                    }
                }
                // Now that the data has been copied we need to include the 8 bits in CT in currentLevelKnownBits
                if (useCascadeTag)
                {
                    currentLevelKnownBits += 8;
                }

                // Repeat anti collision loop until we can transmit all UID bits + BCC and receive a SAK - max 32 iterations.
                selectDone = false;
                while (!selectDone)
                {
                    // Find out how many bits and bytes to send and receive.
                    if (currentLevelKnownBits >= 32)
                    { // All UID bits in this Cascade Level are known. This is a SELECT.
                      //Serial.print(F("SELECT: currentLevelKnownBits=")); Serial.println(currentLevelKnownBits, DEC);
                        buffer[1] = 0x70; // NVB - Number of Valid Bits: Seven whole bytes
                                          // Calculate BCC - Block Check Character
                        buffer[6] = buffer[2] ^ buffer[3] ^ buffer[4] ^ buffer[5];
                        // Calculate CRC_A
                        result = CalculateCRC(buffer, 7, &buffer[7]);
                        if (result != STATUS_OK)
                        {
                            return result;
                        }
                        txLastBits = 0; // 0 => All 8 bits are valid.
                        bufferUsed = 9;
                        // Store response in the last 3 bytes of buffer (BCC and CRC_A - not needed after tx)
                        responseBuffer = buffer[6];
                        responseLength = 3;
                    }
                    else
                    { // This is an ANTICOLLISION.
                      //Serial.print(F("ANTICOLLISION: currentLevelKnownBits=")); Serial.println(currentLevelKnownBits, DEC);
                        txLastBits = currentLevelKnownBits % 8;
                        count = currentLevelKnownBits / 8;  // Number of whole bytes in the UID part.
                        index = 2 + count;                  // Number of whole bytes: SEL + NVB + UIDs
                        buffer[1] = (index << 4) + txLastBits;  // NVB - Number of Valid Bits
                        bufferUsed = index + (txLastBits ? 1 : 0);
                        // Store response in the unused part of buffer
                        responseBuffer = &buffer[index];
                        responseLength = sizeof(buffer) - index;
                    }

                    // Set bit adjustments
                    rxAlign = txLastBits;                                           // Having a separate variable is overkill. But it makes the next line easier to read.
                    TransferToSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.BitFramingReg, (rxAlign << 4) + txLastBits);  // RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]

                    // Transmit the buffer and receive the response.
                    result = PCD_TransceiveData(buffer, bufferUsed, responseBuffer, &responseLength, &txLastBits, rxAlign);
                    if (result == STATUS_COLLISION)
                    { // More than one PICC in the field => collision.
                        byte valueOfCollReg = PCD_ReadRegister(CollReg); // CollReg[7..0] bits are: ValuesAfterColl reserved CollPosNotValid CollPos[4:0]
                        if (valueOfCollReg & 0x20)
                        { // CollPosNotValid
                            return STATUS_COLLISION; // Without a valid collision position we cannot continue
                        }
                        byte collisionPos = valueOfCollReg & 0x1F; // Values 0-31, 0 means bit 32.
                        if (collisionPos == 0)
                        {
                            collisionPos = 32;
                        }
                        if (collisionPos <= currentLevelKnownBits)
                        { // No progress - should not happen 
                            return STATUS_INTERNAL_ERROR;
                        }
                        // Choose the PICC with the bit set.
                        currentLevelKnownBits = collisionPos;
                        count = (currentLevelKnownBits - 1) % 8; // The bit to modify
                        index = 1 + (currentLevelKnownBits / 8) + (count ? 1 : 0); // First byte is index 0.
                        buffer[index] |= (1 << count);
                    }
                    else if (result != STATUS_OK)
                    {
                        return result;
                    }
                    else
                    { // STATUS_OK
                        if (currentLevelKnownBits >= 32)
                        { // This was a SELECT.
                            selectDone = true; // No more anticollision 
                                               // We continue below outside the while.
                        }
                        else
                        { // This was an ANTICOLLISION.
                          // We now have all 32 bits of the UID in this Cascade Level
                            currentLevelKnownBits = 32;
                            // Run loop again to do the SELECT.
                        }
                    }
                } // End of while (!selectDone)

                // We do not check the CBB - it was constructed by us above.

                // Copy the found UID bytes from buffer[] to uid->uidByte[]
                index = (buffer[2] == PICC_CMD_CT) ? 3 : 2; // source index in buffer[]
                bytesToCopy = (buffer[2] == ((byte)MFRC522Registermap.PICC_Command.PICC_CMD_CT)) ? 3 : 4;
                for (count = 0; count < bytesToCopy; count++)
                {
                    uid->uidByte[uidIndex + count] = buffer[index++];
                }

                // Check response SAK (Select Acknowledge)
                if (responseLength != 3 || txLastBits != 0)
                { // SAK must be exactly 24 bits (1 byte + CRC_A).
                    return STATUS_ERROR;
                }
                // Verify CRC_A - do our own calculation and store the control in buffer[2..3] - those bytes are not needed anymore.
                result = PCD_CalculateCRC(responseBuffer, 1, &buffer[2]);
                if (result != STATUS_OK)
                {
                    return result;
                }
                if ((buffer[2] != responseBuffer[1]) || (buffer[3] != responseBuffer[2]))
                {
                    return STATUS_CRC_WRONG;
                }
                if (responseBuffer[0] & 0x04)
                { // Cascade bit set - UID not complete yes
                    cascadeLevel++;
                }
                else
                {
                    uidComplete = true;
                    uid->sak = responseBuffer[0];
                }
            } // End of while (!uidComplete)

            // Set correct uid->size
            uid->size = 3 * cascadeLevel + 1;

            return STATUS_OK;
        } // End PICC_Select()
        #endregion
    }
}