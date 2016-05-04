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
using System.Runtime.InteropServices;

namespace IoTCore {
    class MFRC522 {
        private const Byte DATA_COMMAND_PIN = 0; /* We use GPIO 22 since it's conveniently near the SPI pins */
        private const String READ = "readreg", WRITE = "writereg";

        private GpioController _gpioController;
        private SpiDevice _spiDevice;
        private GpioPin _resetPin;
        public Uid uid = new Uid();

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
                                                                                             /* TPreScaler = TModeReg[3..0]:TPrescalerReg, ie 0x0A9 = 169 => f_timer=40kHz, ie a timer period of 25ms. */                                                                               /* Force 100% ASK modulation */
            TransferToSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.TxASKReg, 0x40);
            /* Set CRC to 0x6363 */
            TransferToSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.ModeReg, 0x3D);
            PCD_AntennaOn();
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
            TransferToSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.CommandReg, (Byte)MFRC522Registermap.PCD_Command.PCD_SoftReset);
            System.Threading.Tasks.Task.Delay(50).Wait();
            while (((TransferToSpi(READ, (Byte)MFRC522Registermap.PCD_Register.CommandReg, 0x00)[1] )  & (1 << 4)) != 0);

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
        public Byte[] TransferToSpi(String operation, Byte register, Byte value){
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
            Byte[] writeBuffer = { register, value };
            Byte[] readBuffer = new Byte[2];
            _spiDevice.TransferFullDuplex(writeBuffer, readBuffer);

            return readBuffer;
        }


        private void PCD_ReadRegister(Byte reg, Byte count, ref Byte[] values, Byte rxAlign) {
            ////< Only bit positions rxAlign..7 in values[0] are updated.
            ////< Byte array to store the values in. - /< The number of bytes to read 
            ////< The register to read from. One of the PCD_Register enums.
            if (count == 0)
                return;
            
            Byte address = (Byte)(0x80 | (reg & 0x7E)); // MSB == 1 is for reading. LSB is not used in address. Datasheet section 8.1.2.3.
            Byte index = 0;
            for(Byte idx = 0; idx < count; ++idx) {

                if (idx == 0 && rxAlign != 0)
                {
                    // Only update bit positions rxAlign..7 in values[0]
                    // Create bit mask for bit positions rxAlign..7
                    Byte mask = 0;
                    for (Byte i = rxAlign; i <= 7; i++)
                    {
                        mask |= (Byte)(1 << i);
                    }
                    // Read value and tell that we want to read the same address again.
                    Byte value = TransferToSpi(READ, address, 0x00)[1];
                    // Apply mask to both current value of values[0] and the new data in value.
                    values[0] = ((Byte)((values[idx] & ~mask) | (value & mask)));
                }
                else
                    values[idx] = (TransferToSpi(READ, address, 0x00)[1]);
                index = idx;
            }
           // values[index + 1] = (TransferToSpi(READ, address, 0x00)[1]);
        } // End PCD_ReadRegister()



        public void getFirmware(){

            Byte version = TransferToSpi(READ, (Byte)MFRC522Registermap.PCD_Register.VersionReg, 0x00 )[1];
            switch (version)
            {
                case 0x88:
                    Debug.WriteLine("Firmware V(smiconductor FM17522 colne)");
                break;
                case 0x90:
                    Debug.WriteLine("Firmware V 0.0");
                break;
                case 0x91:
                    Debug.WriteLine("Firmware V 1.0");
                break;
                case 0x92:
                    Debug.WriteLine("Firmware V 2.0");
                break;
                default:
                    Debug.Write("Unknown");
                break;
            }

        }

        /**
         * Sets the bits given in mask in register reg.
         */
        private void PCD_SetRegisterBitMask(Byte reg,  ///< The register to update. One of the PCD_Register enums.
                                        Byte mask){ ///< The bits to set.

            Byte tmp;
            tmp = TransferToSpi(READ, reg, 0x00)[1];
            TransferToSpi(WRITE, reg, (Byte)(tmp | mask));         // set bit mask
        } // End PCD_SetRegisterBitMask()

        /**
         * Clears the bits given in mask from register reg.
         */
        private void PCD_ClearRegisterBitMask(Byte reg,     ///< The register to update. One of the PCD_Register enums.
                                              Byte mask) {  ///< The bits to clear.
                                      
            Byte tmp;
            tmp = TransferToSpi(READ, reg, 0x00)[1];
            TransferToSpi(WRITE, reg, ((Byte)(tmp & (~mask))));      // clear bit mask
        } // End PCD_ClearRegisterBitMask()
        #endregion

        #region Antanna_Functions
        /**
      * Turns the antenna on by enabling pins TX1 and TX2.
      * After a reset these pins are disabled.
      */
        public void PCD_AntennaOn()
        {
            Byte value = TransferToSpi(READ, (Byte)MFRC522Registermap.PCD_Register.TxControlReg, 0x00)[1];
            if ((value & 0x03) != 0x03)
            {
                TransferToSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.TxControlReg, (Byte)(value | 0x03));
            }
        } // End PCD_AntennaOn()

        /**
         * Turns the antenna off by disabling pins TX1 and TX2.
         */
        public void PCD_AntennaOff()
        {
            PCD_ClearRegisterBitMask((Byte)MFRC522Registermap.PCD_Register.TxControlReg, 0x03);
        } // End PCD_AntennaOff()

        /**
         * Get the current MFRC522 Receiver Gain (RxGain[2:0]) value.
         * See 9.3.3.6 / table 98 in http://www.nxp.com/documents/data_sheet/MFRC522.pdf
         * NOTE: Return value scrubbed with (0x07<<4)=01110000b as RCFfgReg may use reserved bits.
         * 
         * @return Value of the RxGain, scrubbed to the 3 bits used.
         */
        public Byte PCD_GetAntennaGain()
        {
            return (Byte)(TransferToSpi(READ, (Byte)MFRC522Registermap.PCD_Register.RFCfgReg, 0x00)[1] & (0x07 << 4));
        } // End PCD_GetAntennaGain()*/


        public void PCD_SetAntennaGain(Byte mask)
        {
            if (PCD_GetAntennaGain() != mask)
            {                       // only bother if there is a change
                PCD_ClearRegisterBitMask((Byte)MFRC522Registermap.PCD_Register.RFCfgReg, (0x07 << 4));        // clear needed to allow 000 pattern
                PCD_SetRegisterBitMask((Byte)MFRC522Registermap.PCD_Register.RFCfgReg, (Byte)(mask & (0x07 << 4)));   // only set RxGain[2:0] bits
            }
        } // End PCD_SetAntennaGain()
        #endregion

        #region CalculateCRC
        /**
         * FUNCTION NAME : CalulateCRC
         * DESCRIPTION   : 
         * INPUT         : -
         * OUTPUT        : - 
         * NOTE          : Use the CRC coprocessor in the MFRC522 to calculate a CRC_A.
         *                 @return STATUS_OK on success, STATUS_??? otherwise.
         */

        public Byte CalulateCRC(Byte[] Indata, ref Byte[] result) {
            
            TransferToSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.CommandReg, (Byte)MFRC522Registermap.PCD_Command.PCD_Idle);        /* Stop any active command. */

            TransferToSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.DivIrqReg, 0x04);     /* Clear the CRCIRq interrupt request bit */
            TransferToSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.FIFOLevelReg, 0x80);  /* FlushBuffer = 1, FIFO initialization */

            foreach (var b in Indata){
                TransferToSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.FIFODataReg, b);   /* Write data to the FIFO */
            }
            TransferToSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.CommandReg, (Byte)MFRC522Registermap.PCD_Command.PCD_CalcCRC);
            //Indata.Clear();
            UInt16 i = 5000;
            while (true)
            {
                byte r = TransferToSpi(READ, (Byte)MFRC522Registermap.PCD_Register.DivIrqReg, 0x00)[1];
                
                if ((r & 0x04) != 0) 
                    break;
                
                if (--i == 0)
                    return (byte)MFRC522Registermap.StatusCode.STATUS_TIMEOUT;
            }
            
            TransferToSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.CommandReg, (Byte)MFRC522Registermap.PCD_Command.PCD_Idle);        /* Stop calculating CRC for new content in the FIFO. */

            result[0] = (TransferToSpi(READ, (Byte)MFRC522Registermap.PCD_Register.CRCResultRegL, 0x00)[1]);
            result[1] = (TransferToSpi(READ, (Byte)MFRC522Registermap.PCD_Register.CRCResultRegH, 0x00)[1]);

            return (Byte)MFRC522Registermap.StatusCode.STATUS_OK;
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


        public Byte MIFARE_Read(Byte blockAddr, 	    /* MIFARE Classic: The block (0-0xff) number. MIFARE Ultralight: The first page to return data from. */
								ref Byte[] buffer,	/* The buffer to store the data in */
								ref Byte bufferSize){   /* Buffer size, at least 18 bytes. Also number of bytes returned if STATUS_OK. */

            if (buffer == null || bufferSize < 18)
                return (Byte)MFRC522Registermap.StatusCode.STATUS_NO_ROOM;

            //List<Byte> recvData = new List<Byte> { };
            buffer[0] = ((Byte) MFRC522Registermap.PICC_Command.PICC_CMD_MF_READ);
            buffer[1] = (blockAddr);
            // Calculate CRC_A
            Byte result = CalulateCRC(buffer, ref buffer);
            
            if (result != (Byte)MFRC522Registermap.StatusCode.STATUS_OK)
                return result;

            Byte validBits = 0; //dummy data
            // Transmit the buffer and receive the response, validate CRC_A.
            return PCD_TransceiveData(ref buffer, 4, ref buffer, ref bufferSize, ref validBits, 0, true);
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
        public byte MIFARE_Ultralight_Write(Byte blockAddr, 		/* The page (2-15) to write to. */
                                            Byte[] writeData,   /* Buffer size, must be at least 4 bytes. Exactly 4 bytes are written. */
                                            byte bufferSize) {  

            if (writeData == null || bufferSize < 4)
                return (byte)MFRC522Registermap.StatusCode.STATUS_INVALID;

            Byte[] buffer = new Byte[6];
            byte result;
            buffer[0] = ((Byte)MFRC522Registermap.PICC_Command.PICC_CMD_MF_WRITE);
            buffer[1] = (blockAddr);
            for (var idx = 0; idx < writeData.Length; ++idx)
                buffer[idx + 2] = writeData[idx];

            result = PCD_MIFARE_Transceive(ref buffer, 6, true);

            if (result != (byte)MFRC522Registermap.StatusCode.STATUS_OK)
                return result;
            
            return (byte)MFRC522Registermap.StatusCode.STATUS_OK;
        }
        #endregion

        #region PDC_MIFARE_Trasceive
        private Byte PCD_MIFARE_Transceive(
                ref Byte[] sendData,	/* Pointer to the data to transfer to the FIFO. Do NOT include the CRC_A */
                Byte sendLen,
				bool acceptTimeout) {   /* True => A timeout is also success */

            Byte[] cmdBuffer = new Byte[2];
            Byte result;
            

            // Sanity check
            if (sendData == null && sendLen > 18) 
                return (byte)MFRC522Registermap.StatusCode.STATUS_INVALID;

            cmdBuffer = sendData;

            result = CalulateCRC(sendData, ref cmdBuffer);
            if (result != (byte)MFRC522Registermap.StatusCode.STATUS_OK)
                return result;

            sendLen += 2;
            // Transceive the data, store the reply in cmdBuffer[]
            byte waitIRq = 0x30;
            byte cmdBufferSize = 18;
            byte validBits = 0;
            // INNAN 
            //result = PCD_CommunicateWithPICC( (byte)MFRC522Registermap.PCD_Command.PCD_Transceive, 
            //waitIRq, ref sendData, sendLen , ref cmdBuffer, ref cmdBufferSize, ref validBits, 0, true);
            
            // Efter
            result = PCD_CommunicateWithPICC( (byte)MFRC522Registermap.PCD_Command.PCD_Transceive, 
                waitIRq, ref cmdBuffer, sendLen , ref cmdBuffer, ref cmdBufferSize, ref validBits, 0, true);
            //ref List<byte> sendData, byte sendLen, ref List<byte> backData, byte backLen,byte validBits, byte rxAlign, bool checkCRC

            if ((acceptTimeout == true) && (result == (byte)MFRC522Registermap.StatusCode.STATUS_TIMEOUT))
                return (Byte)MFRC522Registermap.StatusCode.STATUS_OK;
            
            if (result != (Byte)MFRC522Registermap.StatusCode.STATUS_OK)
            {
                return result;
            }
            // The PICC must reply with a 4 bit ACK
            if (cmdBufferSize != 1 || validBits != 4)
                return (Byte)MFRC522Registermap.StatusCode.STATUS_ERROR;
           
            if (cmdBuffer[0] != (Byte)MFRC522Registermap.MIFARE_Misc.MF_ACK)
                return (Byte)MFRC522Registermap.StatusCode.STATUS_MIFARE_NACK;
            
            return (Byte)MFRC522Registermap.StatusCode.STATUS_OK;
        } // End PCD_MIFARE_Transceive()

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
            Byte[] bufferATQA = new Byte[2];
            Byte bufferSize = (Byte)bufferATQA.Length;

            Byte result = (byte)(PICC_RequestA(ref bufferATQA, ref bufferSize));
            
            bool b =  (result == (Byte)MFRC522Registermap.StatusCode.STATUS_OK || result == (Byte)MFRC522Registermap.StatusCode.STATUS_COLLISION);
            //Debug.WriteLine(b);
            return b;
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
        /*public bool PICC_ReadCardSerial()
        {
            List<byte> result = PICC_Select(uid, 0);
            return (result[0] == (Byte)MFRC522Registermap.StatusCode.STATUS_OK);
        } // End 
        */
        #endregion

        #region PICC_RequestA
        private byte PICC_RequestA(ref Byte[] bufferATQA,	///< The buffer to store the ATQA (Answer to request) in
								   ref Byte bufferSize) {	///< Buffer size, at least two bytes. Also number of bytes returned if STATUS_OK.
										
            return PICC_REQA_or_WUPA((Byte)MFRC522Registermap.PICC_Command.PICC_CMD_REQA, ref bufferATQA, ref bufferSize);
        } // End PICC_RequestA()

        #endregion

        #region PICC_REQA_or_WUPA
        private Byte PICC_REQA_or_WUPA(Byte command,    		    ///< The command to send - PICC_CMD_REQA or PICC_CMD_WUPA
									   ref Byte[] bufferATQA,	///< The buffer to store the ATQA (Answer to request) in
									   ref Byte bufferSize) {       ///< Buffer size, at least two bytes. Also number of bytes returned if STATUS_OK.

            Byte[] com = { command };							
            Byte validBits, status;

            if (bufferATQA == null || bufferSize < 2) // The ATQA response is 2 bytes long.
                return (Byte)MFRC522Registermap.StatusCode.STATUS_NO_ROOM;
            
            PCD_ClearRegisterBitMask((Byte)MFRC522Registermap.PCD_Register.CollReg, 0x80);        
            // ValuesAfterColl=1 => Bits received after collision are cleared.
            validBits = 7;  // For REQA and WUPA we need the short frame format - transmit only 7 bits of the last (and only) byte. TxLastBits = BitFramingReg[2..0]
            status = PCD_TransceiveData(ref com, 1, ref bufferATQA, ref bufferSize, ref validBits, 0, true);
          
            if (status != (Byte)MFRC522Registermap.StatusCode.STATUS_OK)
                return status;
            
            if (bufferSize != 2 || validBits != 0) // ATQA must be exactly 16 bits.
                return (Byte)MFRC522Registermap.StatusCode.STATUS_ERROR;
            
            return (Byte)MFRC522Registermap.StatusCode.STATUS_OK;
        } // End PICC_REQA_or_WUPA()
        #endregion

        #region PCD_TransceiveData
        private Byte PCD_TransceiveData(ref byte[] sendData,	///< Pointer to the data to transfer to the FIFO.
                                        Byte sendLen,
										ref byte[] backData,	///< NULL or pointer to buffer if data should be read back after executing the command.
										ref Byte backLen,		        ///< In: Max number of bytes to write to *backData. Out: The number of bytes returned.
										ref Byte validBits,	            ///< In/Out: The number of valid bits in the last byte. 0 for 8 valid bits. Default NULL.
										Byte rxAlign,		        ///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
										bool checkCRC) {            ///< In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.

            Byte waitIRq = 0x30;        // RxIRq and IdleIRq
            Byte b =  PCD_CommunicateWithPICC(
                (Byte)MFRC522Registermap.PCD_Command.PCD_Transceive, waitIRq, ref sendData, 
                sendLen, ref backData, ref backLen, ref validBits, rxAlign, checkCRC);
            return b;
        } // End PCD_TransceiveData()
        #endregion

        #region CommunicatteWithPICC
        /**
         * Transfers data to the MFRC522 FIFO, executes a command, waits for completion and transfers data back from the FIFO.
         * CRC validation can only be done if backData and backLen are specified.
         *
         * @return STATUS_OK on success, STATUS_??? otherwise.
         */
     private Byte PCD_CommunicateWithPICC(
                Byte command,               ///< The command to execute. One of the PCD_Command enums.
                Byte waitIRq,               ///< The bits in the ComIrqReg register that signals successful completion of the command.
                ref Byte[] sendData,        ///< Pointer to the data to transfer to the FIFO
                Byte sendLen,
                ref Byte[] backData,        ///< NULL or pointer to buffer if data should be read back after executing the command.
                ref Byte backLen,           ///< In: Max number of bytes to write to *backData. Out: The number of bytes returned.
                ref Byte validBits,         ///< In/Out: The number of valid bits in the last byte. 0 for 8 valid bits.
                Byte rxAlign,               ///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
                bool checkCRC) {            ///< In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.
            
            Byte n, _validBits = 0;

            // Prepare values for BitFramingReg
            //Byte txLastBits = (Byte)(validBits != 0 ? validBits : 0);   //valid bits is not defiend set 0
            Byte bitFraming = (Byte)((rxAlign << 4) + validBits);      // RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]

            TransferToSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.CommandReg, (Byte)MFRC522Registermap.PCD_Command.PCD_Idle);            // Stop any active command.
            TransferToSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.ComIrqReg, 0x7F);                 // Clear all seven interrupt request bits
            PCD_SetRegisterBitMask((Byte)MFRC522Registermap.PCD_Register.FIFOLevelReg, 0x80);         // FlushBuffer = 1, FIFO initialization
            for(int ix = 0; ix < sendLen; ++ix)
                TransferToSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.FIFODataReg, sendData[ix]);  // Write sendData to the FIFO

            TransferToSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.BitFramingReg, bitFraming);       // Bit adjustments
            TransferToSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.CommandReg, command);             // Execute the command
            if (command == (Byte)MFRC522Registermap.PCD_Command.PCD_Transceive)
                PCD_SetRegisterBitMask((Byte)MFRC522Registermap.PCD_Register.BitFramingReg, 0x80);    // StartSend=1, transmission of data starts

            // Wait for the command to complete.
            // In PCD_Init() we set the TAuto flag in TModeReg. This means the timer automatically starts when the PCD stops transmitting.
            // Each iteration of the do-while-loop takes 17.86ms.
            UInt16 i = 2000;
            while (true) {
                n = TransferToSpi(READ, (Byte)MFRC522Registermap.PCD_Register.ComIrqReg, 0x00)[1];    // ComIrqReg[7..0] bits are: Set1 TxIRq RxIRq IdleIRq HiAlertIRq LoAlertIRq ErrIRq TimerIRq
                Debug.WriteLine(n);
                if ((n & waitIRq) != 0) // One of the interrupts that signal success has been set.
                    break;
                if((n & 0x01) != 0)
                    return (byte)MFRC522Registermap.StatusCode.STATUS_TIMEOUT;
                if (--i == 0)
                    return (byte)MFRC522Registermap.StatusCode.STATUS_TIMEOUT;
                
            }

            // Stop now if any errors except collisions were detected.
            Byte errorRegValue = TransferToSpi(READ, (Byte)MFRC522Registermap.PCD_Register.ErrorReg, 0x00)[1]; // ErrorReg[7..0] bits are: WrErr TempErr reserved BufferOvfl CollErr CRCErr ParityErr ProtocolErr
            if ((errorRegValue & 0x13) != 0) // BufferOvfl ParityErr ProtocolErr
                return (Byte)MFRC522Registermap.StatusCode.STATUS_ERROR;
            
            // If the caller wants data back, get it from the MFRC522.
            if (backData != null && backLen != 0) {
                n = TransferToSpi(READ, (Byte)MFRC522Registermap.PCD_Register.FIFOLevelReg, 0x00)[1];         // Number of bytes in the FIFO
                if (n > backLen)
                    return (Byte)MFRC522Registermap.StatusCode.STATUS_NO_ROOM;

                // Number of bytes returned 
                backLen = n;    
                // Get received data from FIFO
                PCD_ReadRegister((Byte)MFRC522Registermap.PCD_Register.FIFODataReg, n, ref backData, rxAlign);

                // RxLastBits[2:0] indicates the number of valid bits in the last received byte. If this value is 000b, the whole byte is valid.
                _validBits = (Byte)(TransferToSpi(READ, (Byte)MFRC522Registermap.PCD_Register.ControlReg, 0x00)[1] & 0x07);
                if (validBits != 0)
                    validBits = _validBits;

            }

            // Tell about collisions
            if ((errorRegValue & 0x08) != 0) // CollErr
                return (Byte)MFRC522Registermap.StatusCode.STATUS_COLLISION;
            

            // Perform CRC_A validation if requested.
            if (backData != null && backLen != 0 && checkCRC) {
                // In this case a MIFARE Classic NAK is not OK.
                if (backLen == 1 && _validBits == 4)
                    return (Byte)MFRC522Registermap.StatusCode.STATUS_MIFARE_NACK;
                
                // We need at least the CRC_A value and all 8 bits of the last byte must be received.
                if (backLen < 2 || _validBits != 0)
                    return (Byte)MFRC522Registermap.StatusCode.STATUS_CRC_WRONG;


                // Verify CRC_A - do our own calculation and store the control in controlBuffer.
                Byte[] controlBuffer = new Byte[2];
                Byte status = CalulateCRC(backData, ref controlBuffer);
                if (status != (Byte)MFRC522Registermap.StatusCode.STATUS_OK)
                    return status;
                
                if ((backData[backLen - 2] != controlBuffer[0]) || (backData[backLen - 1] != controlBuffer[1]))
                    return (Byte)MFRC522Registermap.StatusCode.STATUS_CRC_WRONG;   
            }
            return (Byte)MFRC522Registermap.StatusCode.STATUS_OK;
        } // End PCD_CommunicateWithPICC()

        #endregion

        #region GetPICCType

        public Byte PICC_GetType(Byte sak		///< The SAK byte returned from PICC_Select().
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
        
        public Byte PICC_Select(ref Uid uid,		    ///< Pointer to Uid struct. Normally output, but can also be used to supply a known UID.
						        byte validBits){    ///< The number of known UID bits supplied in *uid. Normally 0. If set you must also supply uid->size.

            bool uidComplete;
            bool selectDone;
            bool useCascadeTag;
            byte cascadeLevel = 1;
            byte count;
            byte index;
            byte uidIndex;                  // The first index in uid->uidByte[] that is used in the current Cascade Level.
            Int16 currentLevelKnownBits;    // The number of known UID bits in the current Cascade Level.
            byte[] buffer = new byte[9];    // The SELECT/ANTICOLLISION commands uses a 7 byte standard frame + 2 bytes CRC_A
            byte bufferUsed;                // The number of bytes used in the buffer, ie the number of bytes to transfer to the FIFO.
            byte rxAlign;                   // Used in BitFramingReg. Defines the bit position for the first bit received.
            byte txLastBits = 0;                // Used in BitFramingReg. The number of valid bits in the last transmitted byte. 
            byte[] responseBuffer = new byte[2];
            byte responseLength = 0, /*responseBuffer,*/ result;

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
                return (Byte)MFRC522Registermap.StatusCode.STATUS_INVALID;
            

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
                        useCascadeTag = validBits != 0 && (uid.size > 4); // When we know that the UID has more than 4 bytes
                        break;

                    case 2:
                        buffer[0] = (Byte)MFRC522Registermap.PICC_Command.PICC_CMD_SEL_CL2;
                        uidIndex = 3;
                        useCascadeTag = validBits != 0 && uid.size > 7; // When we know that the UID has more than 7 bytes
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
                currentLevelKnownBits = (short)(validBits - (8 * uidIndex));
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
                byte bytesToCopy = (byte)(currentLevelKnownBits / 8 + ((currentLevelKnownBits % 8 == 0) ? 1 : 0)); // The number of bytes needed to represent the known bits for this level.
                if (bytesToCopy > 0)
                {
                    byte maxBytes = (byte)(useCascadeTag ? 3 : 4); // Max 4 bytes in each Cascade Level. Only 3 left if we use the Cascade Tag
                    if (bytesToCopy > maxBytes)
                        bytesToCopy = maxBytes;
                    
                    for (count = 0; count < bytesToCopy; count++)
                        buffer[index++] = uid.uidByte[uidIndex + count];
                    
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
                        buffer[6] = (byte)(buffer[2] ^ buffer[3] ^ buffer[4] ^ buffer[5]);
                        // Calculate CRC_A
                        result = CalulateCRC(buffer, ref buffer); //, 7, &buffer[7]
                        if (result != (byte)MFRC522Registermap.StatusCode.STATUS_OK)
                        {
                            return result;
                        }
                        txLastBits = 0; // 0 => All 8 bits are valid.
                        bufferUsed = 9;
                        // Store response in the last 3 bytes of buffer (BCC and CRC_A - not needed after tx)
                        responseBuffer[0] = buffer[6];
                        responseLength = 3;
                    }
                    else{// This is an ANTICOLLISION.
                        txLastBits = (byte)(currentLevelKnownBits % 8);
                        count = (byte)(currentLevelKnownBits / 8);  // Number of whole bytes in the UID part.
                        index = (byte)(2 + count);                  // Number of whole bytes: SEL + NVB + UIDs
                        buffer[1] = (byte)((index << 4) + txLastBits);  // NVB - Number of Valid Bits
                        bufferUsed = (byte)(index + (txLastBits != 0 ? 1 : 0));
                        // Store response in the unused part of buffer
                        responseBuffer[0] = buffer[index];
                        responseLength = (byte)((buffer.Length) - index);
                    }

                    // Set bit adjustments
                    rxAlign = txLastBits;                                           // Having a separate variable is overkill. But it makes the next line easier to read.
                    TransferToSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.BitFramingReg, (byte)((rxAlign << 4) + txLastBits));  // RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]

                    // Transmit the buffer and receive the response.
                    result = PCD_TransceiveData(ref buffer, bufferUsed, ref responseBuffer, ref responseLength, ref txLastBits, rxAlign, true);
                    //ref byte[] sendData, Byte sendLen, ref byte[] backData, ref Byte backLen, ref Byte validBits, Byte rxAlign, bool checkCRC

                    if (result == (byte)MFRC522Registermap.StatusCode.STATUS_COLLISION)
                    { // More than one PICC in the field => collision.
                        byte valueOfCollReg = TransferToSpi(READ, (byte)MFRC522Registermap.PCD_Register.CollReg, 0x00)[1]; // CollReg[7..0] bits are: ValuesAfterColl reserved CollPosNotValid CollPos[4:0]
                        if ((valueOfCollReg & 0x20) != 0)
                        { // CollPosNotValid
                            return (byte)MFRC522Registermap.StatusCode.STATUS_COLLISION; // Without a valid collision position we cannot continue
                        }
                        byte collisionPos = (byte)(valueOfCollReg & 0x1F); // Values 0-31, 0 means bit 32.
                        if (collisionPos == 0)
                        {
                            collisionPos = 32;
                        }
                        if (collisionPos <= currentLevelKnownBits)
                        { // No progress - should not happen 
                            return (byte)MFRC522Registermap.StatusCode.STATUS_INTERNAL_ERROR;
                        }
                        // Choose the PICC with the bit set.
                        currentLevelKnownBits = collisionPos;
                        count = (byte)((currentLevelKnownBits - 1) % 8); // The bit to modify
                        index = (Byte)(1 + (currentLevelKnownBits / 8) + (count != 0  ? 1 : 0)); // First byte is index 0.
                        buffer[index] |= (byte)(1 << count);
                    }
                    else if (result != (byte)MFRC522Registermap.StatusCode.STATUS_OK)
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
                index = (byte)((buffer[2] == (byte)MFRC522Registermap.PICC_Command.PICC_CMD_CT) ? 3 : 2); // source index in buffer[]
                bytesToCopy = (byte)(buffer[2] == (((byte)MFRC522Registermap.PICC_Command.PICC_CMD_CT)) ? 3 : 4);
                for (count = 0; count < bytesToCopy; count++)
                {
                    uid.uidByte[uidIndex + count] = buffer[index++];
                }

                // Check response SAK (Select Acknowledge)
                if (responseLength != 3 || txLastBits != 0)
                // SAK must be exactly 24 bits (1 byte + CRC_A).
                    return (byte)MFRC522Registermap.StatusCode.STATUS_ERROR;
                
                // Verify CRC_A - do our own calculation and store the control in buffer[2..3] - those bytes are not needed anymore.
                result = CalulateCRC(responseBuffer, ref buffer);
                if (result != (byte)MFRC522Registermap.StatusCode.STATUS_OK)
                {
                    return result;
                }
                if ((buffer[2] != responseBuffer[1]) || (buffer[3] != responseBuffer[2]))
                {
                    return (byte)MFRC522Registermap.StatusCode.STATUS_CRC_WRONG;
                }
                if ((responseBuffer[0] & 0x04) != 0)
                { // Cascade bit set - UID not complete yes
                    cascadeLevel++;
                }
                else
                {
                    uidComplete = true;
                    uid.sak = responseBuffer[0];
                }
            } // End of while (!uidComplete)

            // Set correct uid->size
            uid.size = (byte)(3 * cascadeLevel + 1);

            return (byte)MFRC522Registermap.StatusCode.STATUS_OK;
        } // End PICC_Select()
        #endregion
    }
}