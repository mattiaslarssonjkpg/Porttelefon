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
        
        #region openMFRC522
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
                _resetPin = _gpioController.OpenPin(20); //reset pin is now on 38d
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
            TransferToSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.TReloadRegL, 0xD8);   /* TAuto=1; timer starts automatically at the end of the transmission in all communication modes at all speeds */
                                                                                             /* TPreScaler = TModeReg[3..0]:TPrescalerReg, ie 0x0A9 = 169 => f_timer=40kHz, ie a timer period of 25ms. */                                                                               /* Force 100% ASK modulation */
            TransferToSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.TxASKReg, 0x40);
            /* Set CRC to 0x6363 */
            TransferToSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.ModeReg, 0x3D);
            PCD_AntennaOn();
        }
        #endregion

        #region Reset
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
        #endregion

        #region TransferToSpi
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
        #endregion


        #region PCD_ReadRegister

        private void PCD_ReadRegister(byte reg, byte count, ref byte[] values, byte rxAlign) {
            if (count == 0)
                return;

            byte address = (Byte)(0x80 | (reg & 0x7E)); // MSB == 1 is for reading. LSB is not used in address. Datasheet section 8.1.2.3.
            byte index = 0; // Index in values array.

            while (index < count) { 
                values[index] = values[index] = TransferToSpi(READ, reg, 0)[1]; // Read value and tell that we want to read the same address again.
                index++;
            }
            int x = 0; //Just for setting breakpoint
        } // End PCD_ReadRegister()
        #endregion 

        #region getFirmware
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
        #endregion

        #region PCD_SetRegisterBitMask
        /**
         * Sets the bits given in mask in register reg.
         */
        private void PCD_SetRegisterBitMask(Byte reg,  ///< The register to update. One of the PCD_Register enums.
                                        Byte mask){ ///< The bits to set.

            Byte tmp;
            tmp = TransferToSpi(READ, reg, 0x00)[1];
            TransferToSpi(WRITE, reg, (Byte)(tmp | mask));         // set bit mask
        } // End PCD_SetRegisterBitMask()
        #endregion

        #region PCD_ClearRegisterBitMask
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

        public Byte CalulateCRC(ref Byte[] Indata, Byte len, ref Byte[] result) {
            
            TransferToSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.CommandReg, (Byte)MFRC522Registermap.PCD_Command.PCD_Idle);        /* Stop any active command. */

            TransferToSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.DivIrqReg, 0x04);     /* Clear the CRCIRq interrupt request bit */
            PCD_SetRegisterBitMask((Byte)MFRC522Registermap.PCD_Register.FIFOLevelReg, 0x80); /* FlushBuffer = 1, FIFO initialization */

            for (int idx = 0; idx < len; ++idx){
                TransferToSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.FIFODataReg, Indata[idx]);   /* Write data to the FIFO */
            }
            TransferToSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.CommandReg, (Byte)MFRC522Registermap.PCD_Command.PCD_CalcCRC);



            UInt16 i = 5000;
            while (true) {
                byte r = TransferToSpi(READ, (Byte)MFRC522Registermap.PCD_Register.DivIrqReg, 0x00)[1];
                
                if ((r & 0x04) != 0) 
                    break;
                
                if (--i == 0)
                    return (byte)MFRC522Registermap.StatusCode.STATUS_TIMEOUT;
            }
            
            TransferToSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.CommandReg, (Byte)MFRC522Registermap.PCD_Command.PCD_Idle);        /* Stop calculating CRC for new content in the FIFO. */
            
            result[result.Length - 2] = (TransferToSpi(READ, (Byte)MFRC522Registermap.PCD_Register.CRCResultRegL, 0x00)[1]);
            result[result.Length - 1] = (TransferToSpi(READ, (Byte)MFRC522Registermap.PCD_Register.CRCResultRegH, 0x00)[1]);

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
            Byte result = CalulateCRC(ref buffer, 2, ref buffer);
            
            if (result != (Byte)MFRC522Registermap.StatusCode.STATUS_OK)
                return result;

            Byte validBits = 0; //dummy data
            // Transmit the buffer and receive the response, validate CRC_A.
            return PCD_TransceiveData(ref buffer, 4, ref buffer, ref bufferSize, ref validBits, 0, true);
        }
        #endregion

        #region MIFARE_WRITE
        /**
         * Writes 16 bytes to the active PICC.
         * 
         * For MIFARE Classic the sector containing the block must be authenticated before calling this function.
         * 
         * For MIFARE Ultralight the operation is called "COMPATIBILITY WRITE".
         * Even though 16 bytes are transferred to the Ultralight PICC, only the least significant 4 bytes (bytes 0 to 3)
         * are written to the specified address. It is recommended to set the remaining bytes 04h to 0Fh to all logic 0.
         * * 
         * @return STATUS_OK on success, STATUS_??? otherwise.
         */
        public byte MIFARE_Write(byte blockAddr, ref byte[] buffer, byte bufferSize) {
         
            byte result;

            // Sanity check
            if (buffer == null || bufferSize < 16)
                return (Byte)MFRC522Registermap.StatusCode.STATUS_INVALID;

            // Mifare Classic protocol requires two communications to perform a write.
            // Step 1: Tell the PICC we want to write to block blockAddr.
            Byte[] cmdBuffer = new Byte[2];
            cmdBuffer[0] = (Byte)MFRC522Registermap.PICC_Command.PICC_CMD_MF_WRITE;
            cmdBuffer[1] = blockAddr;
            result = PCD_MIFARE_Transceive(ref cmdBuffer, 2, false); // Adds CRC_A and checks that the response is MF_ACK.
            if (result != (Byte)MFRC522Registermap.StatusCode.STATUS_OK)
                return result;
            
            // Step 2: Transfer the data
            result = PCD_MIFARE_Transceive(ref buffer, bufferSize, false); // Adds CRC_A and checks that the response is MF_ACK.
            if(result != (Byte)MFRC522Registermap.StatusCode.STATUS_OK)
                return result;

            return (Byte)MFRC522Registermap.StatusCode.STATUS_OK;
        } // End MIFARE_Write()
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

            result = CalulateCRC(ref cmdBuffer, sendLen, ref cmdBuffer);
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
        public bool PICC_IsNewCardPresent() {
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
        public bool PICC_ReadCardSerial()
        {
            Byte result = PICC_Select(ref uid);
            return (result == (Byte)MFRC522Registermap.StatusCode.STATUS_OK);
        } // End 
        
        #endregion

        #region PICC_RequestA
        private byte PICC_RequestA(ref Byte[] bufferATQA, ref Byte bufferSize) {
										
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
            status = PCD_TransceiveData(ref com, 1, ref bufferATQA, ref bufferSize, ref validBits, 0, false);
          
            if (status != (Byte)MFRC522Registermap.StatusCode.STATUS_OK)
                return status;
            
            if (bufferSize != 2 || validBits != 0) // ATQA must be exactly 16 bits.
                return (Byte)MFRC522Registermap.StatusCode.STATUS_ERROR;
            
            return (Byte)MFRC522Registermap.StatusCode.STATUS_OK;
        } // End PICC_REQA_or_WUPA()
        #endregion

        #region PCD_TransceiveData
        //PCD_TransceiveData(byte *sendData, byte sendLen, byte *backData, byte *backLen, byte *validBits = NULL, byte rxAlign = 0, bool checkCRC = false);
        private Byte PCD_TransceiveData(ref byte[] sendData,	        ///< Pointer to the data to transfer to the FIFO.
                                        Byte sendLen,
										ref byte[] backData,	        ///< NULL or pointer to buffer if data should be read back after executing the command.
										ref Byte backLen,		        ///< In: Max number of bytes to write to *backData. Out: The number of bytes returned.
										ref Byte validBits,	            ///< In/Out: The number of valid bits in the last byte. 0 for 8 valid bits. Default NULL.
										Byte rxAlign = 0,		        ///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
										bool checkCRC = false) {        ///< In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.
            // RxIRq and IdleIRq
            Byte waitIRq = 0x30;        
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
        // StatusCode PCD_CommunicateWithPICC(byte command, byte waitIRq, byte *sendData, byte sendLen, byte *backData = NULL, byte *backLen = NULL, byte *validBits = NULL, byte rxAlign = 0, bool checkCRC = false);
        private Byte PCD_CommunicateWithPICC(Byte command, Byte waitIRq, ref Byte[] sendData, Byte sendLen, ref Byte[] backData, ref Byte backLen, ref Byte validBits, Byte rxAlign = 0, bool checkCRC = false) {
            Byte n, _validBits = 0;
            // Prepare values for BitFramingReg
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
                //Debug.WriteLine(n);
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
                Byte status = CalulateCRC(ref backData, (Byte)(backLen - 2) , ref controlBuffer);
                if (status != (Byte)MFRC522Registermap.StatusCode.STATUS_OK)
                    return status;
                
                if ((backData[backLen - 2] != controlBuffer[0]) || (backData[backLen - 1] != controlBuffer[1]))
                    return (Byte)MFRC522Registermap.StatusCode.STATUS_CRC_WRONG;   
            }
            return (Byte)MFRC522Registermap.StatusCode.STATUS_OK;
        } // End PCD_CommunicateWithPICC()

        #endregion

        #region PICC_GetTypeName

        public String PICC_GetTypeName(Byte piccType) {
            switch (piccType){
                case (Byte)MFRC522Registermap.PICC_Type.PICC_TYPE_ISO_14443_4:
                    return "PICC compliant with ISO/IEC 14443-4";
                case (Byte)MFRC522Registermap.PICC_Type.PICC_TYPE_ISO_18092:
                    return "PICC compliant with ISO/IEC 18092 (NFC)";
                case (Byte)MFRC522Registermap.PICC_Type.PICC_TYPE_MIFARE_MINI:
                    return "MIFARE Mini, 320 bytes";
                case (Byte)MFRC522Registermap.PICC_Type.PICC_TYPE_MIFARE_1K:
                    return "MIFARE 1KB";
                case (Byte)MFRC522Registermap.PICC_Type.PICC_TYPE_MIFARE_4K:
                    return "MIFARE 4KB";
                case (Byte)MFRC522Registermap.PICC_Type.PICC_TYPE_MIFARE_UL:
                    return "MIFARE Ultralight or Ultralight C";
                case (Byte)MFRC522Registermap.PICC_Type.PICC_TYPE_MIFARE_PLUS:
                    return "MIFARE Plus";
                case (Byte)MFRC522Registermap.PICC_Type.PICC_TYPE_TNP3XXX:
                    return "MIFARE TNP3XXX";
                case (Byte)MFRC522Registermap.PICC_Type.PICC_TYPE_NOT_COMPLETE:
                    return "SAK indicates UID is not complete.";
                case (Byte)MFRC522Registermap.PICC_Type.PICC_TYPE_UNKNOWN:
                default:
                    return "Unknown type";
            }
        } // End PICC_GetTypeName()

        #endregion

        #region GetPICCType

        public Byte PICC_GetType(Byte sak) {
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
        
        public Byte PICC_Select(ref Uid uid, byte validBits = 0){
            
            bool uidComplete, selectDone, useCascadeTag;
            Byte cascadeLevel = 1;
            Byte count;
            Byte index;
            Byte uidIndex;                      // The first index in uid->uidByte[] that is used in the current Cascade Level.
            sbyte currentLevelKnownBits;         // The number of known UID bits in the current Cascade Level.
            Byte[] buffer = new byte[9];        // The SELECT/ANTICOLLISION commands uses a 7 byte standard frame + 2 bytes CRC_A
            Byte bufferUsed;                    // The number of bytes used in the buffer, ie the number of bytes to transfer to the FIFO.
            Byte rxAlign;                       // Used in BitFramingReg. Defines the bit position for the first bit received.
            Byte txLastBits = 0;                // Used in BitFramingReg. The number of valid bits in the last transmitted byte. 
            uid.uidByte = new Byte[10];
            Byte[] responseBuffer = new byte[3];
            Byte responseLength = 0, result;

            // Sanity checks
            if (validBits > 80)    
                return (Byte)MFRC522Registermap.StatusCode.STATUS_INVALID;
            
            // Prepare MFRC522
            PCD_ClearRegisterBitMask((Byte)MFRC522Registermap.PCD_Register.CollReg, 0x80);        // ValuesAfterColl=1 => Bits received after collision are cleared.

            // Repeat Cascade Level loop until we have a complete UID.
            uidComplete = false;
            while (!uidComplete) {
                // Set the Cascade Level in the SEL byte, find out if we need to use the Cascade Tag in byte 2.
                switch (cascadeLevel) {
                    case 1:
                        buffer[0] = (Byte)MFRC522Registermap.PICC_Command.PICC_CMD_SEL_CL1;
                        uidIndex = 0;
                        // When we know that the UID has more than 4 bytes
                        useCascadeTag = (validBits != 0) && (uid.size > 4); 
                        break;
                    case 2:
                        buffer[0] = (Byte)MFRC522Registermap.PICC_Command.PICC_CMD_SEL_CL2;
                        uidIndex = 3;
                        // When we know that the UID has more than 7 bytes
                        useCascadeTag = (validBits != 0) && (uid.size > 7); 
                        break;
                    case 3:
                        buffer[0] = (Byte)MFRC522Registermap.PICC_Command.PICC_CMD_SEL_CL3;
                        uidIndex = 6;
                        // Never used in CL3.
                        useCascadeTag = false; 
                        break;
                    default:
                        return (Byte)MFRC522Registermap.StatusCode.STATUS_INTERNAL_ERROR;
                }

                // How many UID bits are known in this Cascade Level?
                currentLevelKnownBits = (sbyte)(validBits - (8 * uidIndex));
                if (currentLevelKnownBits < 0)
                    currentLevelKnownBits = 0;
                
                // Copy the known bits from uid->uidByte[] to buffer[]
                index = 2; // destination index in buffer[]
                if (useCascadeTag)
                    buffer[index++] = (Byte)MFRC522Registermap.PICC_Command.PICC_CMD_CT;

                //Byte bytesToCopy = (Byte)( ((currentLevelKnownBits % 8 != 0) ? 1 : 0)); // The number of bytes needed to represent the known bits for this level.
                Byte bytesToCopy = (Byte)( currentLevelKnownBits / 8 + (currentLevelKnownBits % 8 != 0 ? 1 : 0)); // The number of bytes needed to represent the known bits for this level.
                if (bytesToCopy != 0) {
                    Byte maxBytes = (Byte)(useCascadeTag ? 3 : 4); // Max 4 bytes in each Cascade Level. Only 3 left if we use the Cascade Tag
                    if (bytesToCopy > maxBytes)
                        bytesToCopy = maxBytes;
                    
                    for (count = 0; count < bytesToCopy; ++count)
                        buffer[index++] = uid.uidByte[uidIndex + count];     
                }
                // Now that the data has been copied we need to include the 8 bits in CT in currentLevelKnownBits
                if (useCascadeTag)
                    currentLevelKnownBits += 8;

                // Repeat anti collision loop until we can transmit all UID bits + BCC and receive a SAK - max 32 iterations.
                selectDone = false;
                while (!selectDone) {
                    // Find out how many bits and bytes to send and receive.
                    if (currentLevelKnownBits >= 32) { 
                        // All UID bits in this Cascade Level are known. This is a SELECT.
                        buffer[1] = 0x70; // NVB - Number of Valid Bits: Seven whole bytes
                                          // Calculate BCC - Block Check Character

                        buffer[6] = (Byte)(buffer[2] ^ buffer[3] ^ buffer[4] ^ buffer[5]);
                        // Calculate CRC_A
                        result = CalulateCRC(ref buffer, 7, ref buffer); 
                        if (result != (Byte)MFRC522Registermap.StatusCode.STATUS_OK)
                            return result;
                        
                        txLastBits = 0; // 0 => All 8 bits are valid.
                        bufferUsed = 9;
                        // Store response in the last 3 bytes of buffer (BCC and CRC_A - not needed after tx)
                        responseBuffer = new Byte[3];
                        for (int a = 0; a < 3; ++a) 
                            responseBuffer[a] = buffer[a+6];
                        
                        responseLength = 3;


                        // Set bit adjustments
                        rxAlign = txLastBits; // Having a separate variable is overkill. But it makes the next line easier to read.
                        TransferToSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.BitFramingReg, (Byte)((rxAlign << 4) + txLastBits));  // RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]

                        //Debug.WriteLine("Test: " + (Byte)((rxAlign << 4) + txLastBits));
                        // Transmit the buffer and receive the response.
                        result = PCD_TransceiveData(ref buffer, bufferUsed, ref responseBuffer, ref responseLength, ref txLastBits, rxAlign, false);
                        //ref byte[] sendData, Byte sendLen, ref byte[] backData, ref Byte backLen, ref Byte validBits, Byte rxAlign, bool checkCRC
                        for (int ix = 0; ix < responseBuffer.Length; ++ix)
                            buffer[ix + 6] = responseBuffer[ix];

                    }
                    else
                    {// This is an ANTICOLLISION.
                        txLastBits = (Byte)(currentLevelKnownBits % 8);
                        count = (Byte)(currentLevelKnownBits / 8);      // Number of whole bytes in the UID part.
                        index = (Byte)(2 + count);                      // Number of whole bytes: SEL + NVB + UIDs
                        buffer[1] = (Byte)((index << 4) + txLastBits);  // NVB - Number of Valid Bits
                        bufferUsed = (Byte)(index + (txLastBits != 0 ? 1 : 0));
                        
                        // Store response in the unused part of buffer
                        responseBuffer = new Byte[buffer.Length - index];
                        for(int a = 0; a < responseBuffer.Length; ++a) { 
                            responseBuffer[a] = buffer[a + index];
                        }
                        /*Debug.WriteLine("ANTICOLLISION: ");
                        for (int a = 0; a < responseBuffer.Length; ++a){
                            Debug.WriteLine(responseBuffer[a]);
                        }*/
                        //Byte x = (Byte)(buffer.Length - index);
                        responseLength = (Byte)(buffer.Length - index);


                        // Set bit adjustments
                        rxAlign = txLastBits; // Having a separate variable is overkill. But it makes the next line easier to read.
                        TransferToSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.BitFramingReg, (Byte)((rxAlign << 4) + txLastBits));  // RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]

                        //Debug.WriteLine("Test: " + (Byte)((rxAlign << 4) + txLastBits));
                        // Transmit the buffer and receive the response.
                        result = PCD_TransceiveData(ref buffer, bufferUsed, ref responseBuffer, ref responseLength, ref txLastBits, rxAlign, false);
                        //ref byte[] sendData, Byte sendLen, ref byte[] backData, ref Byte backLen, ref Byte validBits, Byte rxAlign, bool checkCRC
                        for (int ix = index; ix < responseBuffer.Length; ++ix)
                            buffer[ix] = responseBuffer[ix - index];

                    }

                    // Set bit adjustments
                    //rxAlign = txLastBits; // Having a separate variable is overkill. But it makes the next line easier to read.
                    //TransferToSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.BitFramingReg, (Byte)((rxAlign << 4) + txLastBits));  // RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]

                    //Debug.WriteLine("Test: " + (Byte)((rxAlign << 4) + txLastBits));
                    // Transmit the buffer and receive the response.
                    //result = PCD_TransceiveData(ref buffer, bufferUsed, ref responseBuffer, ref responseLength, ref txLastBits, rxAlign, false);
                    //ref byte[] sendData, Byte sendLen, ref byte[] backData, ref Byte backLen, ref Byte validBits, Byte rxAlign, bool checkCRC
                    //for (int ix = index; ix < responseBuffer.Length; ++ix)
                    //    buffer[ix] = responseBuffer[ix - index];

                    if (result == (Byte)MFRC522Registermap.StatusCode.STATUS_COLLISION) { 
                        // More than one PICC in the field => collision.
                        Byte valueOfCollReg = TransferToSpi(READ, (Byte)MFRC522Registermap.PCD_Register.CollReg, 0x00)[1]; // CollReg[7..0] bits are: ValuesAfterColl reserved CollPosNotValid CollPos[4:0]
                        if ((valueOfCollReg & 0x20) != 0) // CollPosNotValid
                            return (Byte)MFRC522Registermap.StatusCode.STATUS_COLLISION; // Without a valid collision position we cannot continue
                        
                        byte collisionPos = (Byte)(valueOfCollReg & 0x1F); // Values 0-31, 0 means bit 32.
                        if (collisionPos == 0)
                            collisionPos = 32;
                        
                        if (collisionPos <= currentLevelKnownBits) // No progress - should not happen 
                            return (Byte)MFRC522Registermap.StatusCode.STATUS_INTERNAL_ERROR;
                        
                        // Choose the PICC with the bit set.
                        currentLevelKnownBits = (sbyte)collisionPos;
                        count = (Byte)((currentLevelKnownBits - 1) % 8); // The bit to modify
                        index = (Byte)(1 + (currentLevelKnownBits / 8) + (count != 0  ? 1 : 0)); // First byte is index 0.
                        buffer[index] |= (Byte)(1 << count);
                    }
                    else if (result != (Byte)MFRC522Registermap.StatusCode.STATUS_OK)
                        return result;
                    
                    else { // STATUS_OK
                        if (currentLevelKnownBits >= 32) // This was a SELECT.
                            // No more anticollision 
                            selectDone = true; 
                            // We continue below outside the while.
                        
                        else // This was an ANTICOLLISION.
                            currentLevelKnownBits = 32;
                        // We now have all 32 bits of the UID in this Cascade Level
                        // Run loop again to do the SELECT.
                    }
                } // End of while (!selectDone)

                // We do not check the CBB - it was constructed by us above.

                // Copy the found UID bytes from buffer[] to uid->uidByte[]
                index = (Byte)((buffer[2] == (Byte)MFRC522Registermap.PICC_Command.PICC_CMD_CT) ? 3 : 2); // source index in buffer[]
                bytesToCopy = (Byte)  ((buffer[2] == ((Byte)MFRC522Registermap.PICC_Command.PICC_CMD_CT)) ? 3 : 4);
                for (count = 0; count < bytesToCopy; ++count)
                    uid.uidByte[uidIndex + count] = buffer[index++];

                // Check response SAK (Select Acknowledge)
                if (responseLength != 3 || txLastBits != 0)
                // SAK must be exactly 24 bits (1 byte + CRC_A).
                    return (byte)MFRC522Registermap.StatusCode.STATUS_ERROR;
                
                // Verify CRC_A - do our own calculation and store the control in buffer[2..3] - those bytes are not needed anymore.
                result = CalulateCRC(ref responseBuffer, 1, ref buffer);
                if (result != (byte)MFRC522Registermap.StatusCode.STATUS_OK)
                    return result;
                //for (int ix = 2; ix < 4; ++ix)
                //    buffer[ix] = responseBuffer[ix-1];

                if ((buffer[buffer.Length-2] != responseBuffer[1]) || (buffer[buffer.Length-1] != responseBuffer[2]))
                    return (byte)MFRC522Registermap.StatusCode.STATUS_CRC_WRONG;
                
                if ((responseBuffer[0] & 0x04) != 0) // Cascade bit set - UID not complete yes
                    cascadeLevel++;
                
                else {
                    uidComplete = true;
                    uid.sak = responseBuffer[0];
                }
            } // End of while (!uidComplete)

            // Set correct uid->size
            uid.size = (byte)(3 * cascadeLevel + 1);

            return (byte)MFRC522Registermap.StatusCode.STATUS_OK;
        } // End PICC_Select()
        #endregion


        //new implemented...
        #region Auth_Key_A

        public Byte PCD_Authenticate(byte command, byte blockAddr, ref MIFARE_Key key, ref Uid uid){
            //key.keyByte = new Byte[(Byte)MFRC522Registermap.MIFARE_Misc.MF_KEY_SIZE];
            Byte waitIRq = 0x10; // IdleIRq
            // Build command buffer
            Byte[] sendData = new Byte[12];
            Byte[] BackData = new Byte[0];
            Byte ValidBytes = 0, backlen = (Byte)BackData.Length;
            sendData[0] = command;
            sendData[1] = blockAddr;

            for (byte i = 0; i < (Byte)MFRC522Registermap.MIFARE_Misc.MF_KEY_SIZE; i++) // 6 key bytes
                sendData[2 + i] = key.keyByte[i];
            
            for (byte i = 0; i < 4; i++) // The first 4 bytes of the UID
                sendData[8 + i] = uid.uidByte[i];

            // Start the authentication.
            Byte b =  PCD_CommunicateWithPICC((Byte)MFRC522Registermap.PCD_Command.PCD_MFAuthent, waitIRq, ref sendData, (Byte)(sendData.Length), ref BackData, ref backlen, ref ValidBytes);
            Byte x = 0;
            return b;
        } // End PCD_Authenticate()
        #endregion

        #region PiCC_Halt
        public Byte PICC_HaltA(){
         
            Byte[] buffer = new Byte[4];
            Byte result, backLen = 0, validBits = 0;
            Byte[] backData = new Byte[0];
            

            // Build command buffer
            buffer[0] = (Byte)MFRC522Registermap.PICC_Command.PICC_CMD_HLTA;
            buffer[1] = 0;
            // Calculate CRC_A
            result = CalulateCRC(ref buffer, 2, ref buffer);
            if (result != (Byte)MFRC522Registermap.StatusCode.STATUS_OK)
                return result;
            

            // Send the command.
            // The standard says:
            //		If the PICC responds with any modulation during a period of 1 ms after the end of the frame containing the
            //		HLTA command, this response shall be interpreted as 'not acknowledge'.
            // We interpret that this way: Only STATUS_TIMEOUT is a success.
            result = PCD_TransceiveData(ref buffer, (Byte)buffer.Length, ref backData, ref backLen, ref validBits);
            if (result == (Byte)MFRC522Registermap.StatusCode.STATUS_TIMEOUT)
                return (Byte)MFRC522Registermap.StatusCode.STATUS_OK;
            
            if (result == (Byte)MFRC522Registermap.StatusCode.STATUS_OK)
                return (Byte)MFRC522Registermap.StatusCode.STATUS_ERROR;
            
            return result;
        } // End PICC_HaltA()


        #endregion
    }
}