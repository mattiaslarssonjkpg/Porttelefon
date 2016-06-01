using IoTCore.RFID_Constans;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace IoTCore.UnitTests
{
    class UnitTest
    {
        private const String WRITE = "writereg", READ = "readreg";
        private MFRC552 mf;

        public UnitTest(MFRC552 _mf) { mf = _mf; }

        /**
         * FUNCTION NAME : UnitTestMFRCReadWriteRegister
         * DESCRIPTION   : Setting registers in the PICC to see if the registers have the value who was written to the register
         * INPUT         : 
         * OUTPUT        : Boolean
         * NOTE          : -
         */

        public async Task<Boolean> UnitTestMFRCReadWriteRegister()
        {
            await mf.openMFRC522();
            bool isRead = true;

            for (int i = 0; i < 15000; i++) {
                var f = mf.TransferToSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.TxControlReg, 0x0A);
                var f1 = mf.TransferToSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.CommandReg, 0x0F);
                var f2 = mf.TransferToSpi(WRITE, (Byte)MFRC522Registermap.PCD_Register.ControlReg, 0x1F);

                var t = mf.TransferToSpi(READ, (Byte)MFRC522Registermap.PCD_Register.TxControlReg, 0x00);
                var t2 = mf.TransferToSpi(READ, (Byte)MFRC522Registermap.PCD_Register.CommandReg, 0x00);
                var t3 = mf.TransferToSpi(READ, (Byte)MFRC522Registermap.PCD_Register.ControlReg, 0x00);
                if (t[0] == 0x0A && t2[0] == 0x0F && t3[0] == 0x1F) { 
                    isRead = false;
                    //break;
                }
            }
            if(isRead == true)
                return true;

            return false;
        }

        /**
        * FUNCTION NAME : UnitTestMFRCReadWriteFifo
        * DESCRIPTION   : Setting the fifo registers and then read the fifo register to see if the data is the same
        * INPUT         : 
        * OUTPUT        : Boolean
        * NOTE          : -
        */

        /*public Boolean UnitTestMFRCReadWriteFifo() {
            //test write to fifo and read fifo buffer
            // This follows directly the steps outlined in 16.1.1
            // 1. Perform a soft reset.
            mf.Reset();

            // 2. Clear the internal buffer by writing 25 bytes of 00h
            byte ZEROES[25] = { 0x00 };
            PCD_SetRegisterBitMask(FIFOLevelReg, 0x80); // flush the FIFO buffer
            PCD_WriteRegister(FIFODataReg, 25, ZEROES); // write 25 bytes of 00h to FIFO
            PCD_WriteRegister(CommandReg, PCD_Mem);     // transfer to internal buffer

            // 3. Enable self-test
            PCD_WriteRegister(AutoTestReg, 0x09);

            // 4. Write 00h to FIFO buffer
            PCD_WriteRegister(FIFODataReg, 0x00);

            // 5. Start self-test by issuing the CalcCRC command
            PCD_WriteRegister(CommandReg, PCD_CalcCRC);

            // 6. Wait for self-test to complete
            word i;
            byte n;
            for (i = 0; i < 0xFF; i++)
            {
                n = PCD_ReadRegister(DivIrqReg);    // DivIrqReg[7..0] bits are: Set2 reserved reserved MfinActIRq reserved CRCIRq reserved reserved
                if (n & 0x04)
                {                       // CRCIRq bit set - calculation done
                    break;
                }
            }
            PCD_WriteRegister(CommandReg, PCD_Idle);        // Stop calculating CRC for new content in the FIFO.

            // 7. Read out resulting 64 bytes from the FIFO buffer.
            byte result[64];
            PCD_ReadRegister(FIFODataReg, 64, result, 0);

            // Auto self-test done
            // Reset AutoTestReg register to be 0 again. Required for normal operation.
            PCD_WriteRegister(AutoTestReg, 0x00);

            // Determine firmware version (see section 9.3.4.8 in spec)
            byte version = PCD_ReadRegister(VersionReg);

            // Pick the appropriate reference values
            const byte* reference;
            switch (version)
            {
                case 0x88:  // Fudan Semiconductor FM17522 clone
                    reference = FM17522_firmware_reference;
                    break;
                case 0x90:  // Version 0.0
                    reference = MFRC522_firmware_referenceV0_0;
                    break;
                case 0x91:  // Version 1.0
                    reference = MFRC522_firmware_referenceV1_0;
                    break;
                case 0x92:  // Version 2.0
                    reference = MFRC522_firmware_referenceV2_0;
                    break;
                default:    // Unknown version
                    return false; // abort test
            }

            // Verify that the results match up to our expectations
            for (i = 0; i < 64; i++)
            {
                if (result[i] != pgm_read_byte(&(reference[i])))
                {
                    return false;
                }
            }

            // Test passed; all is good.
            return true;


            return true;
        }*/
    }
}
