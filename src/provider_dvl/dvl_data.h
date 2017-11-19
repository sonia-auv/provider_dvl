//
// Created by bullshark on 5/11/17.
//

#ifndef PROVIDER_DVL_DVL_DATA_H
#define PROVIDER_DVL_DVL_DATA_H

#pragma pack(push, 1)

typedef struct
{
  uint8_t sync;
  uint8_t hdrSize;
  uint8_t ID;
  uint8_t family;
  uint16_t dataSize;
  uint16_t dataChecksum;
  uint16_t hdrChecksum;
} DVLHeader;

typedef struct
{
  uint32_t beam1VelValid : 1;
  uint32_t beam2VelValid : 1;
  uint32_t beam3VelValid : 1;
  uint32_t beam4VelValid : 1;
  uint32_t beam1DistValid : 1;
  uint32_t beam2DistValid : 1;
  uint32_t beam3DistValid : 1;
  uint32_t beam4DistValid : 1;
  uint32_t beam1FOMValid : 1;
  uint32_t beam2FOMValid : 1;
  uint32_t beam3FOMValid : 1;
  uint32_t beam4FOMValid : 1;
  uint32_t xVelValid : 1;
  uint32_t yVelValid : 1;
  uint32_t z1VelValid : 1;
  uint32_t z2VelValid : 1;
  uint32_t xFOMValid : 1;
  uint32_t yFOMValid : 1;
  uint32_t z1FOMValid : 1;
  uint32_t z2FOMValid : 1;
  uint32_t procIdle3 : 1;
  uint32_t procIdle6 : 1;
  uint32_t procIdle12 : 1;
  uint32_t _empty1 : 5;
  uint32_t wakeupstate : 4;
} DVLstatus_t;

// use of union to acess the bit field as a uint32_t
typedef union
{
    DVLstatus_t bit_field;
    uint32_t integer;
} DVLstatus;

typedef struct
{
  uint8_t version;
  uint8_t offsetOfData;
  uint32_t serialNumber;
  uint8_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t seconds;
  uint16_t microSeconds100;
  uint16_t nBeams;
  uint32_t error;
  DVLstatus status;
  float soundSpeed;
  float temperature;
  float pressure;
/* Beam data */
  float velBeam[4];
  float distBeam[4];
  float fomBeam[4];
  float timeDiff1Beam[4];
  float timeDiff2Beam[4];
  float timeVelEstBeam[4];
/* XYZ data */
  float velX;
  float velY;
  float velZ1;
  float velZ2;
  float fomX;
  float fomY;
  float fomZ1;
  float fomZ2;
  float timeDiff1X;
  float timeDiff1Y;
  float timeDiff1Z1;
  float timeDiff1Z2;
  float timeDiff2X;
  float timeDiff2Y;
  float timeDiff2Z1;
  float timeDiff2Z2;
  float timeVelEstX;
  float timeVelEstY;
  float timeVelEstZ1;
  float timeVelEstZ2;
} DVLData;

typedef struct {
  DVLHeader header;
  DVLData data;
} DVLformat21_t;

#pragma pack(pop)

inline unsigned short calculateChecksum(unsigned short *pData, unsigned short size)
{
  unsigned short checksum = 0xB58C;
  unsigned short nbshorts = (size >> 1);
  int i;
  for (i = 0; i < nbshorts; i++)
  {
    checksum += *pData;
    size -= 2;
    pData++;
  }
  if (size > 0)
  {
    checksum += ((unsigned short)(*pData)) << 8;
  }
  return checksum;
}

#endif //PROVIDER_DVL_DVL_DATA_H
