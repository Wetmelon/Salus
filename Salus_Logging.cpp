// 
// 
// 

#include "Salus_Logging.h"

#define error(s)            (sd.errorHalt(F(s)))
#define SD_CHIPSELECT       (17)
#define SD_SPI_SPEED        (SPI_FULL_SPEED)

SdFat sd;
SdFile file;
SdBaseFile binFile;

// Number of data records in a block.
const uint16_t DATA_DIM = 512 / sizeof(salus_data_t);

//Compute fill so block size is 512 bytes.  FILL_DIM may be zero.
const uint16_t FILL_DIM = 512 - DATA_DIM*sizeof(salus_data_t);

// Maximum file size in blocks.
// The program creates a contiguous file with FILE_BLOCK_COUNT 512 byte blocks.
// This file is flash erased using special SD commands.
// 360000 entries is good for 3600 seconds of logging (60 minutes)
const uint32_t FILE_BLOCK_COUNT = (360000 / DATA_DIM);

// max number of blocks to erase per erase call
uint32_t const ERASE_SIZE = 262144L;

uint32_t bgnBlock, endBlock, blockNum = 0;

struct block_t {
    salus_data_t data[DATA_DIM];
    uint8_t fill[FILL_DIM];
};

block_t block;

void fastLog(){
    if (blockNum == DATA_DIM-1){
        if (!sd.card()->isBusy()){
            if (!sd.card()->writeData((uint8_t*)&block)){
                error("fast write failed");
            }
            blockNum = 0;
        }
        else
            Serial.println("Card BUSY!");
    }
    else
        blockNum++;
}

void startBinLogger(void (*dateTime)(uint16_t *date, uint16_t *time)){

#ifdef LOGGER_DEBUG
    Serial.print("Size of Struct: ");
    Serial.println(sizeof(salus_data_t));
    Serial.print("Data_DIM: ");
    Serial.println(DATA_DIM);
    Serial.print("FILL_DIM: ");
    Serial.println(FILL_DIM);
    Serial.print("Sizeof Block: ");
    Serial.println(sizeof(block_t));
    Serial.println();
#endif

    if (!sd.begin(SD_CHIPSELECT, SD_SPI_SPEED)) {
        sd.initErrorHalt();
    }

    int number = 0;
    char sName[80];

    // Find a filename that hasn't been used already
    do
    {
        sprintf(sName, "Salus_Results_%d.bin", number++);
    } while (sd.exists(sName));

    binFile.close();

    binFile.dateTimeCallback(dateTime);

    if (!binFile.createContiguous(sd.vwd(), sName, 512 * FILE_BLOCK_COUNT)){
        error("createContiguous failed");
    }

    if (!binFile.contiguousRange(&bgnBlock, &endBlock)){
        error("contiguousRange failed");
    }

    // Use SdFat's internal buffer ( ???? )
    uint8_t* cache = (uint8_t*)sd.vol()->cacheClear();
    if (cache == 0) {
        error("cacheClear failed");
    }

    binFile.dateTimeCallbackCancel();

    uint32_t bgnErase = bgnBlock;
    uint32_t endErase;
    while (bgnErase < endBlock) {
        endErase = bgnErase + ERASE_SIZE;
        if (endErase > endBlock) {
            endErase = endBlock;
        }
        if (!sd.card()->erase(bgnErase, endErase)) {
            error("erase failed");
        }
        bgnErase = endErase + 1;
    }


    // Start a multiple block write.
    if (!sd.card()->writeStart(bgnBlock, FILE_BLOCK_COUNT)) {
        error("writeBegin failed");
    }
}

salus_data_t* getBuffer(){
    return &(block.data[blockNum]);
}