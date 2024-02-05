/* Copyright (C) 2013-2016, The Regents of The University of Michigan.
All rights reserved.

This software was developed in the APRIL Robotics Lab under the
direction of Edwin Olson, ebolson@umich.edu. This software may be
available under alternative licensing terms; contact the address above.

An unlimited license is granted to use, adapt, modify, or embed the 2D
barcodes into any medium.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of the Regents of The University of Michigan.
*/

#include <stdlib.h>
#include "apriltag.h"

apriltag_family_t *tag36h12_create()
{
   apriltag_family_t *tf = calloc(1, sizeof(apriltag_family_t));
   tf->name = strdup("tag36h12");
   tf->black_border = 1;
   tf->d = 6;
   tf->h = 12;
   tf->ncodes =250 ;
   tf->codes = calloc(250, sizeof(uint64_t));
tf->codes[0] = 0xd2b63a09dUL;
   tf->codes[1] = 0x6001134e5UL;
   tf->codes[2] = 0x1206fbe72UL;
   tf->codes[3] = 0xff8ad6cb4UL;
   tf->codes[4] = 0x85da9bc49UL;
   tf->codes[5] = 0xb461afe9cUL;
   tf->codes[6] = 0x6db51fe13UL;
   tf->codes[7] = 0x5248c541fUL;
   tf->codes[8] = 0x8f34503UL;
   tf->codes[9] = 0x8ea462eceUL;
   tf->codes[10] = 0xeac2be76dUL;
   tf->codes[11] = 0x1af615c44UL;
   tf->codes[12] = 0xb48a49f27UL;
   tf->codes[13] = 0x2e4e1283bUL;
   tf->codes[14] = 0x78b1f2fa8UL;
   tf->codes[15] = 0x27d34f57eUL;
   tf->codes[16] = 0x89222fff1UL;
   tf->codes[17] = 0x4c1669406UL;
   tf->codes[18] = 0xbf49b3511UL;
   tf->codes[19] = 0xdc191cd5dUL;
   tf->codes[20] = 0x11d7c3f85UL;
   tf->codes[21] = 0x16a130e35UL;
   tf->codes[22] = 0xe29f27effUL;
   tf->codes[23] = 0x428d8ae0cUL;
   tf->codes[24] = 0x90d548477UL;
   tf->codes[25] = 0x2319cbc93UL;
   tf->codes[26] = 0xc3b0c3dfcUL;
   tf->codes[27] = 0x424bccc9UL;
   tf->codes[28] = 0x2a081d630UL;
   tf->codes[29] = 0x762743d96UL;
   tf->codes[30] = 0xd0645bf19UL;
   tf->codes[31] = 0xf38d7fd60UL;
   tf->codes[32] = 0xc6cbf9a10UL;
   tf->codes[33] = 0x3c1be7c65UL;
   tf->codes[34] = 0x276f75e63UL;
   tf->codes[35] = 0x4490a3f63UL;
   tf->codes[36] = 0xda60acd52UL;
   tf->codes[37] = 0x3cc68df59UL;
   tf->codes[38] = 0xab46f9daeUL;
   tf->codes[39] = 0x88d533d78UL;
   tf->codes[40] = 0xb6d62ec21UL;
   tf->codes[41] = 0xb3c02b646UL;
   tf->codes[42] = 0x22e56d408UL;
   tf->codes[43] = 0xac5f5770aUL;
   tf->codes[44] = 0xaaa993f66UL;
   tf->codes[45] = 0x4caa07c8dUL;
   tf->codes[46] = 0x5c9b4f7b0UL;
   tf->codes[47] = 0xaa9ef0e05UL;
   tf->codes[48] = 0x705c5750UL;
   tf->codes[49] = 0xac81f545eUL;
   tf->codes[50] = 0x735b91e74UL;
   tf->codes[51] = 0x8cc35cee4UL;
   tf->codes[52] = 0xe44694d04UL;
   tf->codes[53] = 0xb5e121de0UL;
   tf->codes[54] = 0x261017d0fUL;
   tf->codes[55] = 0xf1d439eb5UL;
   tf->codes[56] = 0xa1a33ac96UL;
   tf->codes[57] = 0x174c62c02UL;
   tf->codes[58] = 0x1ee27f716UL;
   tf->codes[59] = 0x8b1c5ece9UL;
   tf->codes[60] = 0x6a05b0c6aUL;
   tf->codes[61] = 0xd0568dfcUL;
   tf->codes[62] = 0x192d25e5fUL;
   tf->codes[63] = 0x1adbeccc8UL;
   tf->codes[64] = 0xcfec87f00UL;
   tf->codes[65] = 0xd0b9dde7aUL;
   tf->codes[66] = 0x88dcef81eUL;
   tf->codes[67] = 0x445681cb9UL;
   tf->codes[68] = 0xdbb2ffc83UL;
   tf->codes[69] = 0xa48d96df1UL;
   tf->codes[70] = 0xb72cc2e7dUL;
   tf->codes[71] = 0xc295b53fUL;
   tf->codes[72] = 0xf49832704UL;
   tf->codes[73] = 0x9968edc29UL;
   tf->codes[74] = 0x9e4e1af85UL;
   tf->codes[75] = 0x8683e2d1bUL;
   tf->codes[76] = 0x810b45c04UL;
   tf->codes[77] = 0x6ac44bfe2UL;
   tf->codes[78] = 0x645346615UL;
   tf->codes[79] = 0x3990bd598UL;
   tf->codes[80] = 0x1c9ed0f6aUL;
   tf->codes[81] = 0xc26729d65UL;
   tf->codes[82] = 0x83993f795UL;
   tf->codes[83] = 0x3ac05ac5dUL;
   tf->codes[84] = 0x357adff3bUL;
   tf->codes[85] = 0xd5c05565UL;
   tf->codes[86] = 0x2f547ef44UL;
   tf->codes[87] = 0x86c115041UL;
   tf->codes[88] = 0x640fd9e5fUL;
   tf->codes[89] = 0xce08bbcf7UL;
   tf->codes[90] = 0x109bb343eUL;
   tf->codes[91] = 0xc21435c92UL;
   tf->codes[92] = 0x35b4dfce4UL;
   tf->codes[93] = 0x459752cf2UL;
   tf->codes[94] = 0xec915b82cUL;
   tf->codes[95] = 0x51881eed0UL;
   tf->codes[96] = 0x2dda7dc97UL;
   tf->codes[97] = 0x2e0142144UL;
   tf->codes[98] = 0x42e890f99UL;
   tf->codes[99] = 0x9a8856527UL;
   tf->codes[100] = 0x8e80d9d80UL;
   tf->codes[101] = 0x891cbcf34UL;
   tf->codes[102] = 0x25dd82410UL;
   tf->codes[103] = 0x239551d34UL;
   tf->codes[104] = 0x8fe8f0c70UL;
   tf->codes[105] = 0x94106a970UL;
   tf->codes[106] = 0x82609b40cUL;
   tf->codes[107] = 0xfc9caf36UL;
   tf->codes[108] = 0x688181d11UL;
   tf->codes[109] = 0x718613c08UL;
   tf->codes[110] = 0xf1ab7629UL;
   tf->codes[111] = 0xa357bfc18UL;
   tf->codes[112] = 0x4c03b7a46UL;
   tf->codes[113] = 0x204dedce6UL;
   tf->codes[114] = 0xad6300d37UL;
   tf->codes[115] = 0x84cc4cd09UL;
   tf->codes[116] = 0x42160e5c4UL;
   tf->codes[117] = 0x87d2adfa8UL;
   tf->codes[118] = 0x7850e7749UL;
   tf->codes[119] = 0x4e750fc7cUL;
   tf->codes[120] = 0xbf2e5dfdaUL;
   tf->codes[121] = 0xd88324da5UL;
   tf->codes[122] = 0x234b52f80UL;
   tf->codes[123] = 0x378204514UL;
   tf->codes[124] = 0xabdf2ad53UL;
   tf->codes[125] = 0x365e78ef9UL;
   tf->codes[126] = 0x49caa6ca2UL;
   tf->codes[127] = 0x3c39ddf3UL;
   tf->codes[128] = 0xc68c5385dUL;
   tf->codes[129] = 0x5bfcbbf67UL;
   tf->codes[130] = 0x623241e21UL;
   tf->codes[131] = 0xabc90d5ccUL;
   tf->codes[132] = 0x388c6fe85UL;
   tf->codes[133] = 0xda0e2d62dUL;
   tf->codes[134] = 0x10855dfe9UL;
   tf->codes[135] = 0x4d46efd6bUL;
   tf->codes[136] = 0x76ea12d61UL;
   tf->codes[137] = 0x9db377d3dUL;
   tf->codes[138] = 0xeed0efa71UL;
   tf->codes[139] = 0xe6ec3ae2fUL;
   tf->codes[140] = 0x441faee83UL;
   tf->codes[141] = 0xba19c8ff5UL;
   tf->codes[142] = 0x313035eabUL;
   tf->codes[143] = 0x6ce8f7625UL;
   tf->codes[144] = 0x880dab58dUL;
   tf->codes[145] = 0x8d3409e0dUL;
   tf->codes[146] = 0x2be92ee21UL;
   tf->codes[147] = 0xd60302c6cUL;
   tf->codes[148] = 0x469ffc724UL;
   tf->codes[149] = 0x87eebeed3UL;
   tf->codes[150] = 0x42587ef7aUL;
   tf->codes[151] = 0x7a8cc4e52UL;
   tf->codes[152] = 0x76a437650UL;
   tf->codes[153] = 0x999e41ef4UL;
   tf->codes[154] = 0x7d0969e42UL;
   tf->codes[155] = 0xc02baf46bUL;
   tf->codes[156] = 0x9259f3e47UL;
   tf->codes[157] = 0x2116a1dc0UL;
   tf->codes[158] = 0x9f2de4d84UL;
   tf->codes[159] = 0xeffac29UL;
   tf->codes[160] = 0x7b371ff8cUL;
   tf->codes[161] = 0x668339da9UL;
   tf->codes[162] = 0xd010aee3fUL;
   tf->codes[163] = 0x1cd00b4c0UL;
   tf->codes[164] = 0x95070fc3bUL;
   tf->codes[165] = 0xf84c9a770UL;
   tf->codes[166] = 0x38f863d76UL;
   tf->codes[167] = 0x3646ff045UL;
   tf->codes[168] = 0xce1b96412UL;
   tf->codes[169] = 0x7a5d45da8UL;
   tf->codes[170] = 0x14e00ef6cUL;
   tf->codes[171] = 0x5e95abfd8UL;
   tf->codes[172] = 0xb2e9cb729UL;
   tf->codes[173] = 0x36c47dd7UL;
   tf->codes[174] = 0xb8ee97c6bUL;
   tf->codes[175] = 0xe9e8f657UL;
   tf->codes[176] = 0xd4ad2ef1aUL;
   tf->codes[177] = 0x8811c7f32UL;
   tf->codes[178] = 0x47bde7c31UL;
   tf->codes[179] = 0x3adadfb64UL;
   tf->codes[180] = 0x6e5b28574UL;
   tf->codes[181] = 0x33e67cd91UL;
   tf->codes[182] = 0x2ab9fdd2dUL;
   tf->codes[183] = 0x8afa67f2bUL;
   tf->codes[184] = 0xe6a28fc5eUL;
   tf->codes[185] = 0x72049cdbdUL;
   tf->codes[186] = 0xae65dac12UL;
   tf->codes[187] = 0x1251a4526UL;
   tf->codes[188] = 0x1089ab841UL;
   tf->codes[189] = 0xe2f096ee0UL;
   tf->codes[190] = 0xb0caee573UL;
   tf->codes[191] = 0xfd6677e86UL;
   tf->codes[192] = 0x444b3f518UL;
   tf->codes[193] = 0xbe8b3a56aUL;
   tf->codes[194] = 0x680a75cfcUL;
   tf->codes[195] = 0xac02baea8UL;
   tf->codes[196] = 0x97d815e1cUL;
   tf->codes[197] = 0x1d4386e08UL;
   tf->codes[198] = 0x1a14f5b0eUL;
   tf->codes[199] = 0xe658a8d81UL;
   tf->codes[200] = 0xa3868efa7UL;
   tf->codes[201] = 0x3668a9673UL;
   tf->codes[202] = 0xe8fc53d85UL;
   tf->codes[203] = 0x2e2b7edd5UL;
   tf->codes[204] = 0x8b2470f13UL;
   tf->codes[205] = 0xf69795f32UL;
   tf->codes[206] = 0x4589ffc8eUL;
   tf->codes[207] = 0x2e2080c9cUL;
   tf->codes[208] = 0x64265f7dUL;
   tf->codes[209] = 0x3d714dd10UL;
   tf->codes[210] = 0x1692c6ef1UL;
   tf->codes[211] = 0x3e67f2f49UL;
   tf->codes[212] = 0x5041dad63UL;
   tf->codes[213] = 0x1a1503415UL;
   tf->codes[214] = 0x64c18c742UL;
   tf->codes[215] = 0xa72eec35UL;
   tf->codes[216] = 0x1f0f9dc60UL;
   tf->codes[217] = 0xa9559bc67UL;
   tf->codes[218] = 0xf32911d0dUL;
   tf->codes[219] = 0x21c0d4ffcUL;
   tf->codes[220] = 0xe01cef5b0UL;
   tf->codes[221] = 0x4e23a3520UL;
   tf->codes[222] = 0xaa4f04e49UL;
   tf->codes[223] = 0xe1c4fcc43UL;
   tf->codes[224] = 0x208e8f6e8UL;
   tf->codes[225] = 0x8486774a5UL;
   tf->codes[226] = 0x9e98c7558UL;
   tf->codes[227] = 0x2c59fb7dcUL;
   tf->codes[228] = 0x9446a4613UL;
   tf->codes[229] = 0x8292dcc2eUL;
   tf->codes[230] = 0x4d61631UL;
   tf->codes[231] = 0xd05527809UL;
   tf->codes[232] = 0xa0163852dUL;
   tf->codes[233] = 0x8f657f639UL;
   tf->codes[234] = 0xcca6c3e37UL;
   tf->codes[235] = 0xcb136bc7aUL;
   tf->codes[236] = 0xfc5a83e53UL;
   tf->codes[237] = 0x9aa44fc30UL;
   tf->codes[238] = 0xbdec1bd3cUL;
   tf->codes[239] = 0xe020b9f7cUL;
   tf->codes[240] = 0x4b8f35fb0UL;
   tf->codes[241] = 0xb8165f637UL;
   tf->codes[242] = 0x33dc88d69UL;
   tf->codes[243] = 0x10a2f7e4dUL;
   tf->codes[244] = 0xc8cb5ff53UL;
   tf->codes[245] = 0xde259ff6bUL;
   tf->codes[246] = 0x46d070dd4UL;
   tf->codes[247] = 0x32d3b9741UL;
   tf->codes[248] = 0x7075f1c04UL;
   tf->codes[249] = 0x4d58dbea0UL;






   return tf;
}

void tag36h12_destroy(apriltag_family_t *tf)
{
   free(tf->name);
   free(tf->codes);
   free(tf);
}
