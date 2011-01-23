/*
===========================================================================
ARX FATALIS GPL Source Code
Copyright (C) 1999-2010 Arkane Studios SA, a ZeniMax Media company.

This file is part of the Arx Fatalis GPL Source Code ('Arx Fatalis Source Code'). 

Arx Fatalis Source Code is free software: you can redistribute it and/or modify it under the terms of the GNU General Public 
License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

Arx Fatalis Source Code is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied 
warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with Arx Fatalis Source Code.  If not, see 
<http://www.gnu.org/licenses/>.

In addition, the Arx Fatalis Source Code is also subject to certain additional terms. You should have received a copy of these 
additional terms immediately following the terms and conditions of the GNU General Public License which accompanied the Arx 
Fatalis Source Code. If not, please request a copy in writing from Arkane Studios at the address below.

If you have questions concerning this license or the applicable additional terms, you may contact in writing Arkane Studios, c/o 
ZeniMax Media Inc., Suite 120, Rockville, Maryland 20850 USA.
===========================================================================
*/
//////////////////////////////////////////////////////////////////////////////////////
//   @@        @@@        @@@                @@                           @@@@@     //
//   @@@       @@@@@@     @@@     @@        @@@@                         @@@  @@@   //
//   @@@       @@@@@@@    @@@    @@@@       @@@@      @@                @@@@        //
//   @@@       @@  @@@@   @@@  @@@@@       @@@@@@     @@@               @@@         //
//  @@@@@      @@  @@@@   @@@ @@@@@        @@@@@@@    @@@            @  @@@         //
//  @@@@@      @@  @@@@  @@@@@@@@         @@@@ @@@    @@@@@         @@ @@@@@@@      //
//  @@ @@@     @@  @@@@  @@@@@@@          @@@  @@@    @@@@@@        @@ @@@@         //
// @@@ @@@    @@@ @@@@   @@@@@            @@@@@@@@@   @@@@@@@      @@@ @@@@         //
// @@@ @@@@   @@@@@@@    @@@@@@           @@@  @@@@   @@@ @@@      @@@ @@@@         //
// @@@@@@@@   @@@@@      @@@@@@@@@@      @@@    @@@   @@@  @@@    @@@  @@@@@        //
// @@@  @@@@  @@@@       @@@  @@@@@@@    @@@    @@@   @@@@  @@@  @@@@  @@@@@        //
//@@@   @@@@  @@@@@      @@@      @@@@@@ @@     @@@   @@@@   @@@@@@@    @@@@@ @@@@@ //
//@@@   @@@@@ @@@@@     @@@@        @@@  @@      @@   @@@@   @@@@@@@    @@@@@@@@@   //
//@@@    @@@@ @@@@@@@   @@@@             @@      @@   @@@@    @@@@@      @@@@@      //
//@@@    @@@@ @@@@@@@   @@@@             @@      @@   @@@@    @@@@@       @@        //
//@@@    @@@  @@@ @@@@@                          @@            @@@                  //
//            @@@ @@@                           @@             @@        STUDIOS    //
//////////////////////////////////////////////////////////////////////////////////////                                                                                     
//////////////////////////////////////////////////////////////////////////////////////
// HERMESMain
//////////////////////////////////////////////////////////////////////////////////////
//
// Description:
//		HUM...hum...
//
// Updates: (date) (person) (update)
//
// Code: Sébastien Scieux
//
// Copyright (c) 1999 ARKANE Studios SA. All rights reserved
//////////////////////////////////////////////////////////////////////////////////////

#ifndef ARX_HERMES_PAKMANAGER_H
#define ARX_HERMES_PAKMANAGER_H

#include <vector>

class PakFileHandle;
class PakReader;
class PakDirectory;

#include <cstdio> // TODO remove

void * PAK_FileLoadMalloc(const char * name, long * SizeLoadMalloc = NULL);
void * PAK_FileLoadMallocZero(const char * name, long * SizeLoadMalloc = NULL);

bool PAK_AddPak(const char * pakfile);

FILE * PAK_fopen(const char * filename, const char * mode );
std::size_t PAK_fread(void * buffer, std::size_t size, std::size_t count, FILE * stream );
int PAK_fclose(FILE * stream);
long PAK_ftell(FILE * stream);
long PAK_DirectoryExist(const char * name);
long PAK_FileExist(const char * name);
int PAK_fseek(FILE * fic, long offset, int origin);

void PAK_Close();

//-----------------------------------------------------------------------------
class PakManager
{
public:
	std::vector<PakReader*> vLoadPak;
public:
	PakManager();
	~PakManager();

	bool AddPak(const char * pakname);
	bool RemovePak(const char * pakname);
	bool Read(const char * filename, void * buffer);
	void * ReadAlloc(const char * filenme, int * sizeRead);
	int GetSize(const char * filename);
	PakFileHandle * fOpen(const char * filename);
	int fClose(PakFileHandle * fh);
	int fRead(void * buffer, int size, int count, PakFileHandle * fh);
	int fSeek(PakFileHandle * fh, int offset, int whence);
	int fTell(PakFileHandle * fh);
	std::vector<PakDirectory*> * ExistDirectory(const char * name);
	bool ExistFile(const char * name);
};

#endif // ARX_HERMES_PAKMANAGER_H
