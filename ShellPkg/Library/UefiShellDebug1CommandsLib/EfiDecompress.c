/** @file
  Main file for EfiDecompress shell Debug1 function.

  (C) Copyright 2015 Hewlett-Packard Development Company, L.P.<BR>
  Copyright (c) 2005 - 2018, Intel Corporation. All rights reserved.<BR>
  SPDX-License-Identifier: BSD-2-Clause-Patent

**/

#include "UefiShellDebug1CommandsLib.h"
#include <Protocol/Decompress.h>


//
// Decompression algorithm begins here
//
#define UINT8_MAX 0xff
#define BITBUFSIZ 32
#define MAXMATCH  256
#define THRESHOLD 3
#define CODE_BIT  16
#define BAD_TABLE - 1

//
// C: Char&Len Set; P: Position Set; T: exTra Set
//
#define NC      (0xff + MAXMATCH + 2 - THRESHOLD)
#define CBIT    9
#define EFIPBIT 4
#define MAXPBIT 5
#define TBIT    5
#define MAXNP ((1U << MAXPBIT) - 1)
#define NT    (CODE_BIT + 3)
#if NT > MAXNP
#define NPT NT
#else
#define NPT MAXNP
#endif

typedef struct {
  UINT8   *mSrcBase;  // Starting address of compressed data
  UINT8   *mDstBase;  // Starting address of decompressed data
  UINT32  mOutBuf;
  UINT32  mInBuf;

  UINT16  mBitCount;
  UINT32  mBitBuf;
  UINT32  mSubBitBuf;
  UINT16  mBlockSize;
  UINT32  mCompSize;
  UINT32  mOrigSize;

  UINT16  mBadTableFlag;

  UINT16  mLeft[2 * NC - 1];
  UINT16  mRight[2 * NC - 1];
  UINT8   mCLen[NC];
  UINT8   mPTLen[NPT];
  UINT16  mCTable[4096];
  UINT16  mPTTable[256];
} SCRATCH_DATA;

STATIC UINT16 mPbit = EFIPBIT;

/**
  Shift mBitBuf NumOfBits left. Read in NumOfBits of bits from source.

  @param Sd        The global scratch data
  @param NumOfBit  The number of bits to shift and read.
**/
STATIC
VOID
FillBuf2 (
  IN  SCRATCH_DATA  *Sd,
  IN  UINT16        NumOfBits
  )
{
  Sd->mBitBuf = (UINT32) (((UINT64)Sd->mBitBuf) << NumOfBits);

  while (NumOfBits > Sd->mBitCount) {

    Sd->mBitBuf |= (UINT32) (((UINT64)Sd->mSubBitBuf) << (NumOfBits = (UINT16) (NumOfBits - Sd->mBitCount)));

    if (Sd->mCompSize > 0) {
      //
      // Get 1 byte into SubBitBuf
      //
      Sd->mCompSize--;
      Sd->mSubBitBuf  = 0;
      Sd->mSubBitBuf  = Sd->mSrcBase[Sd->mInBuf++];
      Sd->mBitCount   = 8;

    } else {
      //
      // No more bits from the source, just pad zero bit.
      //
      Sd->mSubBitBuf  = 0;
      Sd->mBitCount   = 8;

    }
  }

  Sd->mBitCount = (UINT16) (Sd->mBitCount - NumOfBits);
  Sd->mBitBuf |= Sd->mSubBitBuf >> Sd->mBitCount;
}

/**
  Get NumOfBits of bits out from mBitBuf. Fill mBitBuf with subsequent
  NumOfBits of bits from source. Returns NumOfBits of bits that are
  popped out.

  @param Sd            The global scratch data.
  @param NumOfBits     The number of bits to pop and read.

  @return The bits that are popped out.
**/
STATIC
UINT32
GetBits2 (
  IN  SCRATCH_DATA  *Sd,
  IN  UINT16        NumOfBits
  )
{
  UINT32  OutBits;

  OutBits = (UINT32) (Sd->mBitBuf >> (BITBUFSIZ - NumOfBits));

  FillBuf2 (Sd, NumOfBits);

  return OutBits;
}

/**
  Decodes a position value.

  @param Sd      the global scratch data

  @return The position value decoded.
**/
STATIC
UINT32
DecodeP2 (
  IN  SCRATCH_DATA  *Sd
  )
{
  UINT16  Val;
  UINT32  Mask;
  UINT32  Pos;

  Val = Sd->mPTTable[Sd->mBitBuf >> (BITBUFSIZ - 8)];

  if (Val >= MAXNP) {
    Mask = 1U << (BITBUFSIZ - 1 - 8);

    do {

      if (Sd->mBitBuf & Mask) {
        Val = Sd->mRight[Val];
      } else {
        Val = Sd->mLeft[Val];
      }

      Mask >>= 1;
    } while (Val >= MAXNP);
  }
  //
  // Advance what we have read
  //
  FillBuf2 (Sd, Sd->mPTLen[Val]);

  Pos = Val;
  if (Val > 1) {
    Pos = (UINT32) ((1U << (Val - 1)) + GetBits2 (Sd, (UINT16) (Val - 1)));
  }

  return Pos;
}


/**
  Creates Huffman Code mapping table according to code length array.

  @param Sd        The global scratch data
  @param NumOfChar Number of symbols in the symbol set
  @param BitLen    Code length array
  @param TableBits The width of the mapping table
  @param Table     The table

  @retval 0         - OK.
  @retval BAD_TABLE - The table is corrupted.
**/
STATIC
UINT16
MakeTable2 (
  IN  SCRATCH_DATA  *Sd,
  IN  UINT16        NumOfChar,
  IN  UINT8         *BitLen,
  IN  UINT16        TableBits,
  OUT UINT16        *Table
  )
{
  UINT16  Count[17];
  UINT16  Weight[17];
  UINT16  Start[18];
  UINT16  *Pointer;
  UINT16  Index3;
  UINT16  Index;
  UINT16  Len;
  UINT16  Char;
  UINT16  JuBits;
  UINT16  Avail;
  UINT16  NextCode;
  UINT16  Mask;
  UINT16  MaxTableLength;

  for (Index = 1; Index <= 16; Index++) {
    Count[Index] = 0;
  }

  for (Index = 0; Index < NumOfChar; Index++) {
    if (BitLen[Index] > 16) {
      return (UINT16) BAD_TABLE;
    }
    Count[BitLen[Index]]++;
  }

  Start[1] = 0;

  for (Index = 1; Index <= 16; Index++) {
    Start[Index + 1] = (UINT16) (Start[Index] + (Count[Index] << (16 - Index)));
  }

  if (Start[17] != 0) {
    /*(1U << 16)*/
    return (UINT16) BAD_TABLE;
  }

  JuBits = (UINT16) (16 - TableBits);

  for (Index = 1; Index <= TableBits; Index++) {
    Start[Index] >>= JuBits;
    Weight[Index] = (UINT16) (1U << (TableBits - Index));
  }

  while (Index <= 16) {
    Weight[Index] = (UINT16) (1U << (16 - Index));
    Index++;
  }

  Index = (UINT16) (Start[TableBits + 1] >> JuBits);

  if (Index != 0) {
    Index3 = (UINT16) (1U << TableBits);
    while (Index != Index3) {
      Table[Index++] = 0;
    }
  }

  Avail = NumOfChar;
  Mask  = (UINT16) (1U << (15 - TableBits));
  MaxTableLength = (UINT16) (1U << TableBits);

  for (Char = 0; Char < NumOfChar; Char++) {

    Len = BitLen[Char];
    if (Len == 0 || Len >= 17) {
      continue;
    }

    NextCode = (UINT16) (Start[Len] + Weight[Len]);

    if (Len <= TableBits) {

      if (Start[Len] >= NextCode || NextCode > MaxTableLength){
        return (UINT16) BAD_TABLE;
      }

      for (Index = Start[Len]; Index < NextCode; Index++) {
        Table[Index] = Char;
      }

    } else {

      Index3  = Start[Len];
      Pointer = &Table[Index3 >> JuBits];
      Index   = (UINT16) (Len - TableBits);

      while (Index != 0) {
        if (*Pointer == 0) {
          Sd->mRight[Avail]                     = Sd->mLeft[Avail] = 0;
          *Pointer = Avail++;
        }

        if (Index3 & Mask) {
          Pointer = &Sd->mRight[*Pointer];
        } else {
          Pointer = &Sd->mLeft[*Pointer];
        }

        Index3 <<= 1;
        Index--;
      }

      *Pointer = Char;

    }

    Start[Len] = NextCode;
  }
  //
  // Succeeds
  //
  return 0;
}



/**
  Reads code lengths for the Extra Set or the Position Set

  @param Sd        The global scratch data
  @param nn        Number of symbols
  @param nbit      Number of bits needed to represent nn
  @param Special   The special symbol that needs to be taken care of

  @retval 0         - OK.
  @retval BAD_TABLE - Table is corrupted.
**/
STATIC
UINT16
ReadPTLen2 (
  IN  SCRATCH_DATA  *Sd,
  IN  UINT16        nn,
  IN  UINT16        nbit,
  IN  UINT16        Special
  )
{
  UINT16  Number;
  UINT16  CharC;
  UINT16  Index;
  UINT32  Mask;

  

  Number = (UINT16) GetBits2 (Sd, nbit);

  if (Number == 0) {
    CharC = (UINT16) GetBits2 (Sd, nbit);

    for (Index = 0; Index < 256; Index++) {
      Sd->mPTTable[Index] = CharC;
    }

    for (Index = 0; Index < nn; Index++) {
      Sd->mPTLen[Index] = 0;
    }

    return 0;
  }

  Index = 0;

  while (Index < Number && Index < NPT) {

    CharC = (UINT16) (Sd->mBitBuf >> (BITBUFSIZ - 3));

    if (CharC == 7) {
      Mask = 1U << (BITBUFSIZ - 1 - 3);
      while (Mask & Sd->mBitBuf) {
        Mask >>= 1;
        CharC += 1;
      }
    }

    FillBuf2 (Sd, (UINT16) ((CharC < 7) ? 3 : CharC - 3));

    Sd->mPTLen[Index++] = (UINT8) CharC;

    if (Index == Special) {
      CharC = (UINT16) GetBits2 (Sd, 2);
      CharC--;
      while ((INT16) (CharC) >= 0 && Index < NPT) {
        Sd->mPTLen[Index++] = 0;
        CharC--;
      }
    }
  }

  while (Index < nn && Index < NPT) {
    Sd->mPTLen[Index++] = 0;
  }

  return MakeTable2 (Sd, nn, Sd->mPTLen, 8, Sd->mPTTable);
}

/**
  Reads code lengths for Char&Len Set.

  @param Sd    the global scratch data
**/
STATIC
VOID
ReadCLen2 (
  SCRATCH_DATA  *Sd
  )
{
  UINT16  Number;
  UINT16  CharC;
  UINT16  Index;
  UINT32  Mask;

  Number = (UINT16) GetBits2 (Sd, CBIT);

  if (Number == 0) {
    CharC = (UINT16) GetBits2 (Sd, CBIT);

    for (Index = 0; Index < NC; Index++) {
      Sd->mCLen[Index] = 0;
    }

    for (Index = 0; Index < 4096; Index++) {
      Sd->mCTable[Index] = CharC;
    }

    return ;
  }

  Index = 0;
  while (Index < Number) {

    CharC = Sd->mPTTable[Sd->mBitBuf >> (BITBUFSIZ - 8)];
    if (CharC >= NT) {
      Mask = 1U << (BITBUFSIZ - 1 - 8);

      do {

        if (Mask & Sd->mBitBuf) {
          CharC = Sd->mRight[CharC];
        } else {
          CharC = Sd->mLeft[CharC];
        }

        Mask >>= 1;

      } while (CharC >= NT);
    }
    //
    // Advance what we have read
    //
    FillBuf2 (Sd, Sd->mPTLen[CharC]);

    if (CharC <= 2) {

      if (CharC == 0) {
        CharC = 1;
      } else if (CharC == 1) {
        CharC = (UINT16) (GetBits2 (Sd, 4) + 3);
      } else if (CharC == 2) {
        CharC = (UINT16) (GetBits2 (Sd, CBIT) + 20);
      }

      CharC--;
      while ((INT16) (CharC) >= 0) {
        Sd->mCLen[Index++] = 0;
        CharC--;
      }

    } else {

      Sd->mCLen[Index++] = (UINT8) (CharC - 2);

    }
  }

  while (Index < NC) {
    Sd->mCLen[Index++] = 0;
  }

  MakeTable2 (Sd, NC, Sd->mCLen, 12, Sd->mCTable);

  return ;
}

/**
  Decode a character/length value.

  @param Sd    The global scratch data.

  @return The value decoded.
**/
STATIC
UINT16
DecodeC2 (
  SCRATCH_DATA  *Sd
  )
{
  UINT16  Index2;
  UINT32  Mask;

  if (Sd->mBlockSize == 0) {
    //
    // Starting a new block
    //
    Sd->mBlockSize    = (UINT16) GetBits2 (Sd, 16);
    Sd->mBadTableFlag = ReadPTLen2 (Sd, NT, TBIT, 3);
    if (Sd->mBadTableFlag != 0) {
      return 0;
    }

    ReadCLen2 (Sd);

    Sd->mBadTableFlag = ReadPTLen2 (Sd, MAXNP, mPbit, (UINT16) (-1));
    if (Sd->mBadTableFlag != 0) {
      return 0;
    }
  }

  Sd->mBlockSize--;
  Index2 = Sd->mCTable[Sd->mBitBuf >> (BITBUFSIZ - 12)];

  if (Index2 >= NC) {
    Mask = 1U << (BITBUFSIZ - 1 - 12);

    do {
      if (Sd->mBitBuf & Mask) {
        Index2 = Sd->mRight[Index2];
      } else {
        Index2 = Sd->mLeft[Index2];
      }

      Mask >>= 1;
    } while (Index2 >= NC);
  }
  //
  // Advance what we have read
  //
  FillBuf2 (Sd, Sd->mCLen[Index2]);

  return Index2;
}

/**
  Decode the source data and put the resulting data into the destination buffer.

  @param Sd            The global scratch data
 **/
STATIC
VOID
Decode2 (
  SCRATCH_DATA  *Sd
  )
{
  UINT16  BytesRemain;
  UINT32  DataIdx;
  UINT16  CharC;

  BytesRemain = (UINT16) (-1);

  DataIdx     = 0;

  for (;;) {
    CharC = DecodeC2 (Sd);
    if (Sd->mBadTableFlag != 0) {
      return ;
    }

    if (CharC < 256) {
      //
      // Process an Original character
      //
      Sd->mDstBase[Sd->mOutBuf++] = (UINT8) CharC;
      if (Sd->mOutBuf >= Sd->mOrigSize) {
        return ;
      }

    } else {
      //
      // Process a Pointer
      //
      CharC       = (UINT16) (CharC - (UINT8_MAX + 1 - THRESHOLD));

      BytesRemain = CharC;

      DataIdx     = Sd->mOutBuf - DecodeP2 (Sd) - 1;

      BytesRemain--;
      while ((INT16) (BytesRemain) >= 0) {
        if (Sd->mOutBuf >= Sd->mOrigSize) {
          return ;
        }
        if (DataIdx >= Sd->mOrigSize) {
          Sd->mBadTableFlag = (UINT16) BAD_TABLE;
          return ;
        }
        Sd->mDstBase[Sd->mOutBuf++] = Sd->mDstBase[DataIdx++];

        BytesRemain--;
      }
      //
      // Once mOutBuf is fully filled, directly return
      //
      if (Sd->mOutBuf >= Sd->mOrigSize) {
        return ;
      }
    }
  }

  return ;
}

/**
  The implementation of EFI_DECOMPRESS_PROTOCOL.GetInfo().

  @param Source      The source buffer containing the compressed data.
  @param SrcSize     The size of source buffer
  @param DstSize     The size of destination buffer.
  @param ScratchSize The size of scratch buffer.

  @retval EFI_SUCCESS           - The size of destination buffer and the size of scratch buffer are successfully retrieved.
  @retval EFI_INVALID_PARAMETER - The source data is corrupted
**/
EFI_STATUS
GetInfo_vuln (
  IN      VOID    *Source,
  IN      UINT32  SrcSize,
  OUT     UINT32  *DstSize,
  OUT     UINT32  *ScratchSize
  )
{
  UINT8 *Src;
  UINT32 CompSize;

  *ScratchSize  = sizeof (SCRATCH_DATA);

  Src           = Source;
  if (SrcSize < 8) {
    return EFI_INVALID_PARAMETER;
  }

  CompSize = Src[0] + (Src[1] << 8) + (Src[2] << 16) + (Src[3] << 24);
  *DstSize = Src[4] + (Src[5] << 8) + (Src[6] << 16) + (Src[7] << 24);

  if (SrcSize < CompSize + 8 || (CompSize + 8) < 8) {
    return EFI_INVALID_PARAMETER;
  }

  return EFI_SUCCESS;
}

/**
  The implementation Efi and Tiano Decompress().

  @param Source      - The source buffer containing the compressed data.
  @param SrcSize     - The size of source buffer
  @param Destination - The destination buffer to store the decompressed data
  @param DstSize     - The size of destination buffer.
  @param Scratch     - The buffer used internally by the decompress routine. This  buffer is needed to store intermediate data.
  @param ScratchSize - The size of scratch buffer.

  @retval EFI_SUCCESS           - Decompression is successful
  @retval EFI_INVALID_PARAMETER - The source data is corrupted
**/
EFI_STATUS
Decompress_Vuln (
  IN      VOID    *Source,
  IN      UINT32  SrcSize,
  IN OUT  VOID    *Destination,
  IN      UINT32  DstSize,
  IN OUT  VOID    *Scratch,
  IN      UINT32  ScratchSize
  )
{
  UINT32        Index;
  UINT32        CompSize;
  UINT32        OrigSize;
  EFI_STATUS    Status;
  SCRATCH_DATA  *Sd;
  UINT8         *Src;
  UINT8         *Dst;

  Status  = EFI_SUCCESS;
  Src     = Source;
  Dst     = Destination;

  if (ScratchSize < sizeof (SCRATCH_DATA)) {
    return EFI_INVALID_PARAMETER;
  }

  Sd = (SCRATCH_DATA *) Scratch;

  if (SrcSize < 8) {
    return EFI_INVALID_PARAMETER;
  }

  CompSize  = Src[0] + (Src[1] << 8) + (Src[2] << 16) + (Src[3] << 24);
  OrigSize  = Src[4] + (Src[5] << 8) + (Src[6] << 16) + (Src[7] << 24);

  if (SrcSize < CompSize + 8 || (CompSize + 8) < 8) {
    return EFI_INVALID_PARAMETER;
  }

  if (DstSize != OrigSize) {
    return EFI_INVALID_PARAMETER;
  }

  Src = Src + 8;

  for (Index = 0; Index < sizeof (SCRATCH_DATA); Index++) {
    ((UINT8 *) Sd)[Index] = 0;
  }

  Sd->mSrcBase  = Src;
  Sd->mDstBase  = Dst;
  Sd->mCompSize = CompSize;
  Sd->mOrigSize = OrigSize;

  //
  // Fill the first BITBUFSIZ bits
  //
  FillBuf2 (Sd, BITBUFSIZ);

  //
  // Decompress it
  //
  Decode2 (Sd);

  if (Sd->mBadTableFlag != 0) {
    //
    // Something wrong with the source
    //
    Status = EFI_INVALID_PARAMETER;
  }

  return Status;
}


/**
  Function for 'decompress' command.

  @param[in] ImageHandle  Handle to the Image (NULL if Internal).
  @param[in] SystemTable  Pointer to the System Table (NULL if Internal).
**/
SHELL_STATUS
EFIAPI
ShellCommandRunEfiDecompress (
  IN EFI_HANDLE        ImageHandle,
  IN EFI_SYSTEM_TABLE  *SystemTable
  )
{
  EFI_STATUS               Status;
  LIST_ENTRY               *Package;
  CHAR16                   *ProblemParam;
  SHELL_STATUS             ShellStatus;
  SHELL_FILE_HANDLE        InFileHandle;
  SHELL_FILE_HANDLE        OutFileHandle;
  UINT32                   OutSize;
  UINTN                    OutSizeTemp;
  VOID                     *OutBuffer;
  UINTN                    InSize;
  VOID                     *InBuffer;
  CHAR16                   *InFileName;
  CONST CHAR16             *OutFileName;
  UINT64                   Temp64Bit;
  UINT32                   ScratchSize;
  VOID                     *ScratchBuffer;
  EFI_DECOMPRESS_PROTOCOL  *Decompress;
  CONST CHAR16             *TempParam;

  InFileName    = NULL;
  OutFileName   = NULL;
  OutSize       = 0;
  ScratchSize   = 0;
  ShellStatus   = SHELL_SUCCESS;
  Status        = EFI_SUCCESS;
  OutBuffer     = NULL;
  InBuffer      = NULL;
  ScratchBuffer = NULL;
  InFileHandle  = NULL;
  OutFileHandle = NULL;
  Decompress    = NULL;

  //
  // initialize the shell lib (we must be in non-auto-init...)
  //
  Status = ShellInitialize ();
  ASSERT_EFI_ERROR (Status);

  Status = CommandInit ();
  ASSERT_EFI_ERROR (Status);

  //
  // parse the command line
  //
  Status = ShellCommandLineParse (EmptyParamList, &Package, &ProblemParam, TRUE);
  if (EFI_ERROR (Status)) {
    if ((Status == EFI_VOLUME_CORRUPTED) && (ProblemParam != NULL)) {
      ShellPrintHiiEx (-1, -1, NULL, STRING_TOKEN (STR_GEN_PROBLEM), gShellDebug1HiiHandle, L"efidecompress", ProblemParam);
      FreePool (ProblemParam);
      ShellStatus = SHELL_INVALID_PARAMETER;
    } else {
      ASSERT (FALSE);
    }
  } else {
    if (ShellCommandLineGetCount (Package) > 3) {
      ShellPrintHiiEx (-1, -1, NULL, STRING_TOKEN (STR_GEN_TOO_MANY), gShellDebug1HiiHandle, L"efidecompress");
      ShellStatus = SHELL_INVALID_PARAMETER;
    } else if (ShellCommandLineGetCount (Package) < 3) {
      ShellPrintHiiEx (-1, -1, NULL, STRING_TOKEN (STR_GEN_TOO_FEW), gShellDebug1HiiHandle, L"efidecompress");
      ShellStatus = SHELL_INVALID_PARAMETER;
    } else {
      TempParam = ShellCommandLineGetRawValue (Package, 1);
      ASSERT (TempParam != NULL);
      InFileName  = ShellFindFilePath (TempParam);
      OutFileName = ShellCommandLineGetRawValue (Package, 2);
      if (InFileName == NULL) {
        ShellPrintHiiEx (-1, -1, NULL, STRING_TOKEN (STR_FILE_FIND_FAIL), gShellDebug1HiiHandle, L"efidecompress", TempParam);
        ShellStatus = SHELL_NOT_FOUND;
      } else {
        if (ShellIsDirectory (InFileName) == EFI_SUCCESS) {
          ShellPrintHiiEx (-1, -1, NULL, STRING_TOKEN (STR_FILE_NOT_DIR), gShellDebug1HiiHandle, L"efidecompress", InFileName);
          ShellStatus = SHELL_INVALID_PARAMETER;
        }

        if (ShellIsDirectory (OutFileName) == EFI_SUCCESS) {
          ShellPrintHiiEx (-1, -1, NULL, STRING_TOKEN (STR_FILE_NOT_DIR), gShellDebug1HiiHandle, L"efidecompress", OutFileName);
          ShellStatus = SHELL_INVALID_PARAMETER;
        }

        if (ShellStatus == SHELL_SUCCESS) {
          Status = ShellOpenFileByName (InFileName, &InFileHandle, EFI_FILE_MODE_READ, 0);
          if (EFI_ERROR (Status)) {
            ShellPrintHiiEx (-1, -1, NULL, STRING_TOKEN (STR_GEN_FILE_OPEN_FAIL), gShellDebug1HiiHandle, L"efidecompress", ShellCommandLineGetRawValue (Package, 1));
            ShellStatus = SHELL_NOT_FOUND;
          }
        }

        if (ShellStatus == SHELL_SUCCESS) {
          Status = FileHandleGetSize (InFileHandle, &Temp64Bit);
          ASSERT_EFI_ERROR (Status);
          if (!EFI_ERROR (Status)) {
            ASSERT (Temp64Bit <= (UINT32)(-1));
            InSize   = (UINTN)Temp64Bit;
            InBuffer = AllocateZeroPool (InSize);
          }

          if (InBuffer == NULL) {
            Status = EFI_OUT_OF_RESOURCES;
          } else {
            Status = gEfiShellProtocol->ReadFile (InFileHandle, &InSize, InBuffer);
            ASSERT_EFI_ERROR (Status);

            Status = gBS->LocateProtocol (&gEfiDecompressProtocolGuid, NULL, (VOID **)&Decompress);
            ASSERT_EFI_ERROR (Status);

            Status = Decompress->GetInfo (Decompress, InBuffer, (UINT32)InSize, &OutSize, &ScratchSize);
          }

          if (EFI_ERROR (Status) || (OutSize == 0)) {
            ShellPrintHiiEx (-1, -1, NULL, STRING_TOKEN (STR_EFI_DECOMPRESS_NOPE), gShellDebug1HiiHandle, InFileName);
            ShellStatus = SHELL_NOT_FOUND;
          } else {
            Status = ShellOpenFileByName (OutFileName, &OutFileHandle, EFI_FILE_MODE_READ|EFI_FILE_MODE_WRITE|EFI_FILE_MODE_CREATE, 0);
            if (EFI_ERROR (Status)) {
              ShellPrintHiiEx (-1, -1, NULL, STRING_TOKEN (STR_FILE_OPEN_FAIL), gShellDebug1HiiHandle, ShellCommandLineGetRawValue (Package, 2), Status);
              ShellStatus = SHELL_NOT_FOUND;
            } else {
              OutBuffer     = AllocateZeroPool (OutSize);
              ScratchBuffer = AllocateZeroPool (ScratchSize);
              if ((OutBuffer == NULL) || (ScratchBuffer == NULL)) {
                Status = EFI_OUT_OF_RESOURCES;
              } else {
                //Status = Decompress->Decompress (Decompress, InBuffer, (UINT32)InSize, OutBuffer, OutSize, ScratchBuffer, ScratchSize);
                Status = Decompress_Vuln (InBuffer, (UINT32)InSize, OutBuffer, OutSize, ScratchBuffer, ScratchSize);
                
              }
            }
          }

          if (EFI_ERROR (Status)) {
            ShellPrintHiiEx (-1, -1, NULL, STRING_TOKEN (STR_EFI_DECOMPRESS_FAIL), gShellDebug1HiiHandle, Status);
            ShellStatus = ((Status == EFI_OUT_OF_RESOURCES) ? SHELL_OUT_OF_RESOURCES : SHELL_DEVICE_ERROR);
          } else {
            OutSizeTemp = OutSize;
            Status      = gEfiShellProtocol->WriteFile (OutFileHandle, &OutSizeTemp, OutBuffer);
            OutSize     = (UINT32)OutSizeTemp;
            if (EFI_ERROR (Status)) {
              ShellPrintHiiEx (-1, -1, NULL, STRING_TOKEN (STR_FILE_WRITE_FAIL), gShellDebug1HiiHandle, L"efidecompress", OutFileName, Status);
              ShellStatus = SHELL_DEVICE_ERROR;
            }
          }
        }
      }
    }

    ShellCommandLineFreeVarList (Package);
  }

  if (InFileHandle != NULL) {
    gEfiShellProtocol->CloseFile (InFileHandle);
  }

  if (OutFileHandle != NULL) {
    gEfiShellProtocol->CloseFile (OutFileHandle);
  }

  SHELL_FREE_NON_NULL (InFileName);
  SHELL_FREE_NON_NULL (InBuffer);
  SHELL_FREE_NON_NULL (OutBuffer);
  SHELL_FREE_NON_NULL (ScratchBuffer);

  return (ShellStatus);
}
