// generate_test_data.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <io.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <direct.h>
#include <stdio.h>
#include <errno.h>

#pragma pack(1)

struct toc_entry 
{
    int offset;
    int size;
    int a;
    int b;
    int c;
    char name[12];
};

struct Tpk
{
    //char name[8];
    struct TInfo
    {
        char Magic[8];
        int Version;
        int Type;
    }Info;
    struct TMinVersion
    {
        int PK;
        int PPA;
        int RD1;
        int RD2;
        int ISW;
        int KI;
        int PAU;
        int PAS;
    }MinVersion;
    int BootSWD;
    struct KeyInfo
    {
        int ID;
        int Type;
        int Rights;
        char PublicKey[264];
    };
    KeyInfo RootKey;
    int KeysNbr;
    KeyInfo AuxKey[6];
    int KILimitation;
    //char Spare[128-8]; ///<OMAPSign used 128 but sample 2nd seems to use 120
    char Spare[128]; ///<OMAPSign used 128 but sample 2nd seems to use 120
    struct Signature
    {
        char SignerInfo[16];
        int SignatureInfo;
        int KeyID;
        char RSASignature[256];
    } RSAsign;
};

FILE *log;

void print(char*name, void *t, size_t size) 
{
    if (size <= 4) 
    {
        fprintf (log,"%0*X %s\n", 2 * size, *(int*)t, name); 
             printf ("%0*X %s\n", 2 * size, *(int*)t, name);
    }
    else
    {
        unsigned char *d = (unsigned char*)t;
        fprintf (log,"%s:", name); 
             //printf ("%s:", name);
        for (size_t i = 0; i < size; i++, d++)
        {
            if (i % 32 == 0)
            {
                fprintf (log,"\n"); 
                     //printf ("\n");
            }
            else if (i % 8 == 0)
            {
                fprintf (log,"."); 
                //printf (".");
            }
            fprintf (log,"%02X", *d); 
                 //printf ("%02X", *d);
        }
        fprintf (log,"\n"); 
             //printf ("\n");
    }
}
template <typename T>
void print(char*name, T&t) 
{
    print(name, &t, sizeof t);
}
#define PRINT(member) print(#member,member)

template <typename T>
void writeKeyInfo (char *in_name,T &key, char *format, int a, int b)
{
       char out_name[200];
       char name[50];
       sprintf(name,format,a,b);
       strcpy(out_name,in_name);
       strcat(out_name,".");
       strcat(out_name,name);
       //fprintf (log,"id:%08X type:%08X rights:%08X %s\n", key.ID, key.Type, key.Rights, name);
       //      printf ("id:%08X type:%08X rights:%08X %s\n", key.ID, key.Type, key.Rights, name);
       fprintf (log,"%s:\n", name);
       printf ("%s:\n", name);
       PRINT(key.ID);
       PRINT(key.Type);
       PRINT(key.Rights);
       PRINT(key.PublicKey);
       FILE *out = fopen(out_name, "wb");
       assert(out);
       int r = fwrite(&key,1,sizeof key,out);
       assert(r==sizeof key);
       fclose(out);
}

void disassemble (char *in_name)
{
    assert(in_name);
    FILE *in = fopen(in_name, "rb");
    char log_name[200];
    strcpy(log_name,in_name);
    strcat(log_name,".log");
    log = fopen(log_name, "w");
    assert(log);
    assert(in);

    toc_entry toc[20]; ///<Probably more than there will ever bee
    int t;
    for(t = 0; t < sizeof toc/sizeof *toc; t++)
    {
        int r = fread(toc+t,1,sizeof *toc,in);
        assert(r ==sizeof*toc);
        int j = 0;
        for (j = 0; j < sizeof *toc; j++)
        {
            if (((char*)(toc+t))[j] != -1)
                break;
        }
        if (j == sizeof *toc)
            break;
        fprintf (log,"@%08X #%08X %08X %08X %08X %s\n", toc[t].offset, toc[t].size, toc[t].a, toc[t].b, toc[t].c, toc[t].name);
             printf ("@%08X #%08X %08X %08X %08X %s\n", toc[t].offset, toc[t].size, toc[t].a, toc[t].b, toc[t].c, toc[t].name);
    }

    Tpk *pk = 0;
    int s = sizeof *pk;
    for (int i = 0; i < t; i++)
    {
       char out_name[200];
       strcpy(out_name,in_name);
       strcat(out_name,".");
       strcat(out_name,toc[i].name);
       FILE *out = fopen(out_name, "wb");
       assert(out);
       fseek(in,toc[i].offset,SEEK_SET);
       void*data = malloc(toc[i].size);
       assert(data);
       size_t r = fread(data,1,toc[i].size,in);
       assert(r==toc[i].size);
       r = fwrite(data,1,toc[i].size,out);
       assert(r==toc[i].size);
       fclose(out);
       if (!strcmp(toc[i].name,"KEYS"))
       {
            pk = ((Tpk*)data);
            //PRINT(pk->name);
            PRINT(pk->Info.Magic);
            PRINT(pk->Info.Version);
            PRINT(pk->Info.Type);
            PRINT(pk->MinVersion.PK);
            PRINT(pk->MinVersion.PPA);
            PRINT(pk->MinVersion.RD1);
            PRINT(pk->MinVersion.RD2);
            PRINT(pk->MinVersion.ISW);
            PRINT(pk->MinVersion.KI);
            PRINT(pk->MinVersion.PAU);
            PRINT(pk->MinVersion.PAS);
            PRINT(pk->BootSWD);
            writeKeyInfo (in_name,pk->RootKey, "RootKey", -1, -1);
            PRINT(pk->KeysNbr);
            //int s = pk->KeysNbr;
            int s = sizeof pk->AuxKey / sizeof *pk->AuxKey;
            for (int k = 0; k < s; k++)
            {
                assert(k < sizeof pk->AuxKey / sizeof *pk->AuxKey);
                writeKeyInfo (in_name,pk->AuxKey[k], "Key.index-%d.id-%08X", k, pk->AuxKey[k].ID);
            }
            PRINT(pk->KILimitation);
            PRINT(pk->Spare);
            PRINT(pk->RSAsign.SignerInfo);
            PRINT(pk->RSAsign.SignatureInfo);
            PRINT(pk->RSAsign.KeyID);
            PRINT(pk->RSAsign.RSASignature);
       }
       else 
       {
           print(toc[i].name, data, toc[i].size);
       }
    }
}

void my_mkdir(char *exe)
{
   //char buf[_MAX_PATH];
   const char * const test_data = "test_data";

   //const char *backspace = strrchr(exe, '\\');
   //assert(backspace);
   //int len = backspace - exe + 1;
   //strncpy_s(buf, sizeof buf, exe, len);
   //strcat_s(buf, sizeof buf, test_data);

   int error = _chdir(test_data);
   if (error == -1)
   {
       error = _mkdir(test_data);
       assert(!error);
       error = _chdir(test_data);
   }
   assert(!error);
}

#define KB(x)  (x * 1024)
#define MB(x)  KB(x * 1024)

void generate_byte_number_file(char * name, unsigned int size)
{
    FILE *f;
    errno_t error;
    error = fopen_s(&f, name, "wb");
    assert(!error);
	  for (unsigned int i = 0; i < size; i++) {
		  fputc(i & 0xFF, f);
    }
	  fclose(f);
}
void generate_number_file(char * name, unsigned int size)
{
    FILE *f;
    errno_t error;
    error = fopen_s(&f, name, "wb");
    assert(!error);
	  for (unsigned int i = 0; i < size; i+=4) {
      fputc((i >> 24) & 0xFF, f);
      fputc((i >> 16) & 0xFF, f);
      fputc((i >> 8) & 0xFF, f);
      fputc(i & 0xFF, f);
    }
	  fclose(f);
}

int main(int argc, char* argv[])
{
    if (argc == 1 || !strcmp(argv[1],"patterns"))
    {
        my_mkdir(argv[0]);

        FILE *f;
        errno_t error;

        error = fopen_s(&f, "pattern1.bin","wb"); //pattern_256
        assert(!error);
        for (int i = 0; i < 256; i++) {
            fputc(i & 0xFF, f);
        }
        fclose(f);

        error = fopen_s(&f, "pattern2.bin","wb"); //reverse_256
        assert(!error);
        for (int i = 0; i < 256; i++) {
            fputc((255-i) & 0xFF, f);
        }
        fclose(f);

        error = fopen_s(&f, "pattern4.bin","wb"); //== pattern_1K
        assert(!error);
        for (int i = 0; i < 1024; i++) {
            fputc(i & 0xFF, f);
        }
        fclose(f);

        error = fopen_s(&f, "pattern5.bin","wb"); //pattern_1M_258K
        assert(!error);
	      for (int i = 0; i < (10 * (64 * 2048) + 2048); i++) {
		      fputc(i & 0xFF, f);
        }
	      fclose(f);

        generate_number_file("pattern_1K.bin", KB(1));
        generate_number_file("pattern_10K.bin", KB(10));
        generate_number_file("pattern_100K.bin", KB(100));
        generate_number_file("pattern_1M.bin", MB(1));     //0x00100000
        generate_number_file("pattern_10M.bin", MB(10));
    #ifdef _DEBUG
        ///@todo need some comments here for the odd sized files
        //generate_number_file("pattern_63M_1023K.bin", MB(63) + KB(1023)); //0x03fffc00
        //generate_number_file("pattern_64M.bin", MB(64));   //0x04000000
        generate_number_file("pattern_100M.bin", MB(100));
        generate_number_file("pattern_128M.bin", MB(128)); //0x08000000
        generate_number_file("pattern_256M.bin", MB(256)); //0x10000000
        //generate_number_file("pattern_512M.bin", MB(512)); //0x20000000
        generate_number_file("pattern_128M_4B.bin", MB(128) + 4);
        generate_number_file("pattern_256M_4B.bin", MB(256) + 4);
        generate_number_file("pattern_512M_4B.bin", MB(512) + 4);
        generate_number_file("pattern_512M_5BB.bin", MB(512) - 5 * KB(128));
    /*
        generate_number_file("pattern_128M_1B.bin", MB(128) + 1);
        generate_number_file("pattern_256M_1B.bin", MB(256) + 1);
        generate_number_file("pattern_512M_1B.bin", MB(512) + 1);
        generate_number_file("pattern_124M.bin", MB(124));
        generate_number_file("pattern_252M.bin", MB(252));
        generate_number_file("pattern_508M.bin", MB(508));
    */
    #endif
    }
    else if (!strcmp(argv[1],"disassemble"))
    {
        disassemble (argv[2]);
    }
    return 0;
}
