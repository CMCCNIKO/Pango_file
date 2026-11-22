//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 未注明
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: OV5640 RGB565 摄像头的 257 条有效寄存器表，由 donor reg_config.v 提取并保留原 donor 索引。
// Module Name: (include) ov5640_reg_table.vh
// File Name: source/hdmi/ov5640_reg_table.vh
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// OV5640 RGB565 摄像头的 257 条有效寄存器表，由 donor reg_config.v 提取并保留原 donor 索引。
// 包含软件复位、时钟、输出格式与图像参数设置，供 SCCB 控制器逐项写入；16 位地址需高字节先发。
//
// Dependencies: 由 ov5640_sccb_init.v 在寄存器 case 内 include。
//
// Revision:
// 2026-11-01 / Header 1.00 - 补充本文件具体功能、依赖、工程版本；保留功能逻辑。
// 原创建日期、功能修订、编译指令和版权保留在下方。
// Additional Comments:
// Company/Engineer 仅标注本次用途说明整理；下方原作者和第三方版权继续有效。
// 保留 donor reg_config.v 的来源及索引注释。
// 当前用途见本说明；下方原始注释作为历史记录保留。
//////////////////////////////////////////////////////////////////////////////////

// OV5640 donor register sequence: valid entries extracted from reg_config.v.
// 16-bit register address is transmitted high byte first.
	10'd0: begin reg_addr=16'h3103; reg_data=8'h11; end // donor index 0
	10'd1: begin reg_addr=16'h3008; reg_data=8'h82; end // donor index 1
	10'd2: begin reg_addr=16'h3008; reg_data=8'h42; end // donor index 102
	10'd3: begin reg_addr=16'h3103; reg_data=8'h03; end // donor index 103
	10'd4: begin reg_addr=16'h3017; reg_data=8'hff; end // donor index 104
	10'd5: begin reg_addr=16'h3018; reg_data=8'hff; end // donor index 105
	10'd6: begin reg_addr=16'h3034; reg_data=8'h1A; end // donor index 106
	10'd7: begin reg_addr=16'h3037; reg_data=8'h13; end // donor index 107
	10'd8: begin reg_addr=16'h3108; reg_data=8'h01; end // donor index 108
	10'd9: begin reg_addr=16'h3630; reg_data=8'h36; end // donor index 109
	10'd10: begin reg_addr=16'h3631; reg_data=8'h0e; end // donor index 110
	10'd11: begin reg_addr=16'h3632; reg_data=8'he2; end // donor index 111
	10'd12: begin reg_addr=16'h3633; reg_data=8'h12; end // donor index 112
	10'd13: begin reg_addr=16'h3621; reg_data=8'he0; end // donor index 113
	10'd14: begin reg_addr=16'h3704; reg_data=8'ha0; end // donor index 114
	10'd15: begin reg_addr=16'h3703; reg_data=8'h5a; end // donor index 115
	10'd16: begin reg_addr=16'h3715; reg_data=8'h78; end // donor index 116
	10'd17: begin reg_addr=16'h3717; reg_data=8'h01; end // donor index 117
	10'd18: begin reg_addr=16'h370b; reg_data=8'h60; end // donor index 118
	10'd19: begin reg_addr=16'h3705; reg_data=8'h1a; end // donor index 119
	10'd20: begin reg_addr=16'h3905; reg_data=8'h02; end // donor index 120
	10'd21: begin reg_addr=16'h3906; reg_data=8'h10; end // donor index 121
	10'd22: begin reg_addr=16'h3901; reg_data=8'h0a; end // donor index 122
	10'd23: begin reg_addr=16'h3731; reg_data=8'h12; end // donor index 123
	10'd24: begin reg_addr=16'h3600; reg_data=8'h08; end // donor index 124
	10'd25: begin reg_addr=16'h3601; reg_data=8'h33; end // donor index 125
	10'd26: begin reg_addr=16'h302d; reg_data=8'h60; end // donor index 126
	10'd27: begin reg_addr=16'h3620; reg_data=8'h52; end // donor index 127
	10'd28: begin reg_addr=16'h371b; reg_data=8'h20; end // donor index 128
	10'd29: begin reg_addr=16'h471c; reg_data=8'h50; end // donor index 129
	10'd30: begin reg_addr=16'h3a13; reg_data=8'h43; end // donor index 130
	10'd31: begin reg_addr=16'h3a18; reg_data=8'h00; end // donor index 131
	10'd32: begin reg_addr=16'h3a19; reg_data=8'hf8; end // donor index 132
	10'd33: begin reg_addr=16'h3635; reg_data=8'h13; end // donor index 133
	10'd34: begin reg_addr=16'h3636; reg_data=8'h03; end // donor index 134
	10'd35: begin reg_addr=16'h3634; reg_data=8'h40; end // donor index 135
	10'd36: begin reg_addr=16'h3622; reg_data=8'h01; end // donor index 136
	10'd37: begin reg_addr=16'h3c01; reg_data=8'h34; end // donor index 137
	10'd38: begin reg_addr=16'h3c04; reg_data=8'h28; end // donor index 138
	10'd39: begin reg_addr=16'h3c05; reg_data=8'h98; end // donor index 139
	10'd40: begin reg_addr=16'h3c06; reg_data=8'h00; end // donor index 140
	10'd41: begin reg_addr=16'h3c07; reg_data=8'h08; end // donor index 141
	10'd42: begin reg_addr=16'h3c08; reg_data=8'h00; end // donor index 142
	10'd43: begin reg_addr=16'h3c09; reg_data=8'h1c; end // donor index 143
	10'd44: begin reg_addr=16'h3c0a; reg_data=8'h9c; end // donor index 144
	10'd45: begin reg_addr=16'h3c0b; reg_data=8'h40; end // donor index 145
	10'd46: begin reg_addr=16'h3810; reg_data=8'h00; end // donor index 146
	10'd47: begin reg_addr=16'h3811; reg_data=8'h10; end // donor index 147
	10'd48: begin reg_addr=16'h3812; reg_data=8'h00; end // donor index 148
	10'd49: begin reg_addr=16'h3708; reg_data=8'h64; end // donor index 149
	10'd50: begin reg_addr=16'h4001; reg_data=8'h02; end // donor index 150
	10'd51: begin reg_addr=16'h4005; reg_data=8'h1a; end // donor index 151
	10'd52: begin reg_addr=16'h3000; reg_data=8'h00; end // donor index 152
	10'd53: begin reg_addr=16'h3004; reg_data=8'hff; end // donor index 153
	10'd54: begin reg_addr=16'h300e; reg_data=8'h58; end // donor index 154
	10'd55: begin reg_addr=16'h302e; reg_data=8'h00; end // donor index 155
	10'd56: begin reg_addr=16'h4300; reg_data=8'h61; end // donor index 156
	10'd57: begin reg_addr=16'h501f; reg_data=8'h01; end // donor index 157
	10'd58: begin reg_addr=16'h440e; reg_data=8'h00; end // donor index 158
	10'd59: begin reg_addr=16'h5000; reg_data=8'ha7; end // donor index 159
	10'd60: begin reg_addr=16'h3a0f; reg_data=8'h30; end // donor index 160
	10'd61: begin reg_addr=16'h3a10; reg_data=8'h28; end // donor index 161
	10'd62: begin reg_addr=16'h3a1b; reg_data=8'h30; end // donor index 162
	10'd63: begin reg_addr=16'h3a1e; reg_data=8'h26; end // donor index 163
	10'd64: begin reg_addr=16'h3a11; reg_data=8'h60; end // donor index 164
	10'd65: begin reg_addr=16'h3a1f; reg_data=8'h14; end // donor index 165
	10'd66: begin reg_addr=16'h5800; reg_data=8'h23; end // donor index 166
	10'd67: begin reg_addr=16'h5801; reg_data=8'h14; end // donor index 167
	10'd68: begin reg_addr=16'h5802; reg_data=8'h0f; end // donor index 168
	10'd69: begin reg_addr=16'h5803; reg_data=8'h0f; end // donor index 169
	10'd70: begin reg_addr=16'h5804; reg_data=8'h12; end // donor index 170
	10'd71: begin reg_addr=16'h5805; reg_data=8'h26; end // donor index 171
	10'd72: begin reg_addr=16'h5806; reg_data=8'h0c; end // donor index 172
	10'd73: begin reg_addr=16'h5807; reg_data=8'h08; end // donor index 173
	10'd74: begin reg_addr=16'h5808; reg_data=8'h05; end // donor index 174
	10'd75: begin reg_addr=16'h5809; reg_data=8'h05; end // donor index 175
	10'd76: begin reg_addr=16'h580a; reg_data=8'h08; end // donor index 176
	10'd77: begin reg_addr=16'h580b; reg_data=8'h0d; end // donor index 177
	10'd78: begin reg_addr=16'h580c; reg_data=8'h08; end // donor index 178
	10'd79: begin reg_addr=16'h580d; reg_data=8'h03; end // donor index 179
	10'd80: begin reg_addr=16'h580e; reg_data=8'h00; end // donor index 180
	10'd81: begin reg_addr=16'h580f; reg_data=8'h00; end // donor index 181
	10'd82: begin reg_addr=16'h5810; reg_data=8'h03; end // donor index 182
	10'd83: begin reg_addr=16'h5811; reg_data=8'h09; end // donor index 183
	10'd84: begin reg_addr=16'h5812; reg_data=8'h07; end // donor index 184
	10'd85: begin reg_addr=16'h5813; reg_data=8'h03; end // donor index 185
	10'd86: begin reg_addr=16'h5814; reg_data=8'h00; end // donor index 186
	10'd87: begin reg_addr=16'h5815; reg_data=8'h01; end // donor index 187
	10'd88: begin reg_addr=16'h5816; reg_data=8'h03; end // donor index 188
	10'd89: begin reg_addr=16'h5817; reg_data=8'h08; end // donor index 189
	10'd90: begin reg_addr=16'h5818; reg_data=8'h0d; end // donor index 190
	10'd91: begin reg_addr=16'h5819; reg_data=8'h08; end // donor index 191
	10'd92: begin reg_addr=16'h581a; reg_data=8'h05; end // donor index 192
	10'd93: begin reg_addr=16'h581b; reg_data=8'h06; end // donor index 193
	10'd94: begin reg_addr=16'h581c; reg_data=8'h08; end // donor index 194
	10'd95: begin reg_addr=16'h581d; reg_data=8'h0e; end // donor index 195
	10'd96: begin reg_addr=16'h581e; reg_data=8'h29; end // donor index 196
	10'd97: begin reg_addr=16'h581f; reg_data=8'h17; end // donor index 197
	10'd98: begin reg_addr=16'h5820; reg_data=8'h11; end // donor index 198
	10'd99: begin reg_addr=16'h5821; reg_data=8'h11; end // donor index 199
	10'd100: begin reg_addr=16'h5822; reg_data=8'h15; end // donor index 200
	10'd101: begin reg_addr=16'h5823; reg_data=8'h28; end // donor index 201
	10'd102: begin reg_addr=16'h5824; reg_data=8'h46; end // donor index 202
	10'd103: begin reg_addr=16'h5825; reg_data=8'h26; end // donor index 203
	10'd104: begin reg_addr=16'h5826; reg_data=8'h08; end // donor index 204
	10'd105: begin reg_addr=16'h5827; reg_data=8'h26; end // donor index 205
	10'd106: begin reg_addr=16'h5828; reg_data=8'h64; end // donor index 206
	10'd107: begin reg_addr=16'h5829; reg_data=8'h26; end // donor index 207
	10'd108: begin reg_addr=16'h582a; reg_data=8'h24; end // donor index 208
	10'd109: begin reg_addr=16'h582b; reg_data=8'h22; end // donor index 209
	10'd110: begin reg_addr=16'h582c; reg_data=8'h24; end // donor index 210
	10'd111: begin reg_addr=16'h582d; reg_data=8'h24; end // donor index 211
	10'd112: begin reg_addr=16'h582e; reg_data=8'h06; end // donor index 212
	10'd113: begin reg_addr=16'h582f; reg_data=8'h22; end // donor index 213
	10'd114: begin reg_addr=16'h5830; reg_data=8'h40; end // donor index 214
	10'd115: begin reg_addr=16'h5831; reg_data=8'h42; end // donor index 215
	10'd116: begin reg_addr=16'h5832; reg_data=8'h24; end // donor index 216
	10'd117: begin reg_addr=16'h5833; reg_data=8'h26; end // donor index 217
	10'd118: begin reg_addr=16'h5834; reg_data=8'h24; end // donor index 218
	10'd119: begin reg_addr=16'h5835; reg_data=8'h22; end // donor index 219
	10'd120: begin reg_addr=16'h5836; reg_data=8'h22; end // donor index 220
	10'd121: begin reg_addr=16'h5837; reg_data=8'h26; end // donor index 221
	10'd122: begin reg_addr=16'h5838; reg_data=8'h44; end // donor index 222
	10'd123: begin reg_addr=16'h5839; reg_data=8'h24; end // donor index 223
	10'd124: begin reg_addr=16'h583a; reg_data=8'h26; end // donor index 224
	10'd125: begin reg_addr=16'h583b; reg_data=8'h28; end // donor index 225
	10'd126: begin reg_addr=16'h583c; reg_data=8'h42; end // donor index 226
	10'd127: begin reg_addr=16'h583d; reg_data=8'hce; end // donor index 227
	10'd128: begin reg_addr=16'h5180; reg_data=8'hff; end // donor index 228
	10'd129: begin reg_addr=16'h5181; reg_data=8'hf2; end // donor index 229
	10'd130: begin reg_addr=16'h5182; reg_data=8'h00; end // donor index 230
	10'd131: begin reg_addr=16'h5183; reg_data=8'h14; end // donor index 231
	10'd132: begin reg_addr=16'h5184; reg_data=8'h25; end // donor index 232
	10'd133: begin reg_addr=16'h5185; reg_data=8'h24; end // donor index 233
	10'd134: begin reg_addr=16'h5186; reg_data=8'h09; end // donor index 234
	10'd135: begin reg_addr=16'h5187; reg_data=8'h09; end // donor index 235
	10'd136: begin reg_addr=16'h5188; reg_data=8'h09; end // donor index 236
	10'd137: begin reg_addr=16'h5189; reg_data=8'h75; end // donor index 237
	10'd138: begin reg_addr=16'h518a; reg_data=8'h54; end // donor index 238
	10'd139: begin reg_addr=16'h518b; reg_data=8'he0; end // donor index 239
	10'd140: begin reg_addr=16'h518c; reg_data=8'hb2; end // donor index 240
	10'd141: begin reg_addr=16'h518d; reg_data=8'h42; end // donor index 241
	10'd142: begin reg_addr=16'h518e; reg_data=8'h3d; end // donor index 242
	10'd143: begin reg_addr=16'h518f; reg_data=8'h56; end // donor index 243
	10'd144: begin reg_addr=16'h5190; reg_data=8'h46; end // donor index 244
	10'd145: begin reg_addr=16'h5191; reg_data=8'hf8; end // donor index 245
	10'd146: begin reg_addr=16'h5192; reg_data=8'h04; end // donor index 246
	10'd147: begin reg_addr=16'h5193; reg_data=8'h70; end // donor index 247
	10'd148: begin reg_addr=16'h5194; reg_data=8'hf0; end // donor index 248
	10'd149: begin reg_addr=16'h5195; reg_data=8'hf0; end // donor index 249
	10'd150: begin reg_addr=16'h5196; reg_data=8'h03; end // donor index 250
	10'd151: begin reg_addr=16'h5197; reg_data=8'h01; end // donor index 251
	10'd152: begin reg_addr=16'h5198; reg_data=8'h04; end // donor index 252
	10'd153: begin reg_addr=16'h5199; reg_data=8'h12; end // donor index 253
	10'd154: begin reg_addr=16'h519a; reg_data=8'h04; end // donor index 254
	10'd155: begin reg_addr=16'h519b; reg_data=8'h00; end // donor index 255
	10'd156: begin reg_addr=16'h519c; reg_data=8'h06; end // donor index 256
	10'd157: begin reg_addr=16'h519d; reg_data=8'h82; end // donor index 257
	10'd158: begin reg_addr=16'h519e; reg_data=8'h38; end // donor index 258
	10'd159: begin reg_addr=16'h5480; reg_data=8'h01; end // donor index 259
	10'd160: begin reg_addr=16'h5481; reg_data=8'h08; end // donor index 260
	10'd161: begin reg_addr=16'h5482; reg_data=8'h14; end // donor index 261
	10'd162: begin reg_addr=16'h5483; reg_data=8'h28; end // donor index 262
	10'd163: begin reg_addr=16'h5484; reg_data=8'h51; end // donor index 263
	10'd164: begin reg_addr=16'h5485; reg_data=8'h65; end // donor index 264
	10'd165: begin reg_addr=16'h5486; reg_data=8'h71; end // donor index 265
	10'd166: begin reg_addr=16'h5487; reg_data=8'h7d; end // donor index 266
	10'd167: begin reg_addr=16'h5488; reg_data=8'h87; end // donor index 267
	10'd168: begin reg_addr=16'h5489; reg_data=8'h91; end // donor index 268
	10'd169: begin reg_addr=16'h548a; reg_data=8'h9a; end // donor index 269
	10'd170: begin reg_addr=16'h548b; reg_data=8'haa; end // donor index 270
	10'd171: begin reg_addr=16'h548c; reg_data=8'hb8; end // donor index 271
	10'd172: begin reg_addr=16'h548d; reg_data=8'hcd; end // donor index 272
	10'd173: begin reg_addr=16'h548e; reg_data=8'hdd; end // donor index 273
	10'd174: begin reg_addr=16'h548f; reg_data=8'hea; end // donor index 274
	10'd175: begin reg_addr=16'h5490; reg_data=8'h1d; end // donor index 275
	10'd176: begin reg_addr=16'h5381; reg_data=8'h1e; end // donor index 276
	10'd177: begin reg_addr=16'h5382; reg_data=8'h5b; end // donor index 277
	10'd178: begin reg_addr=16'h5383; reg_data=8'h08; end // donor index 278
	10'd179: begin reg_addr=16'h5384; reg_data=8'h0a; end // donor index 279
	10'd180: begin reg_addr=16'h5385; reg_data=8'h7e; end // donor index 280
	10'd181: begin reg_addr=16'h5386; reg_data=8'h88; end // donor index 281
	10'd182: begin reg_addr=16'h5387; reg_data=8'h7c; end // donor index 282
	10'd183: begin reg_addr=16'h5388; reg_data=8'h6c; end // donor index 283
	10'd184: begin reg_addr=16'h5389; reg_data=8'h10; end // donor index 284
	10'd185: begin reg_addr=16'h538a; reg_data=8'h01; end // donor index 285
	10'd186: begin reg_addr=16'h538b; reg_data=8'h98; end // donor index 286
	10'd187: begin reg_addr=16'h5580; reg_data=8'h06; end // donor index 287
	10'd188: begin reg_addr=16'h5583; reg_data=8'h40; end // donor index 288
	10'd189: begin reg_addr=16'h5584; reg_data=8'h10; end // donor index 289
	10'd190: begin reg_addr=16'h5589; reg_data=8'h10; end // donor index 290
	10'd191: begin reg_addr=16'h558a; reg_data=8'h00; end // donor index 291
	10'd192: begin reg_addr=16'h558b; reg_data=8'hf8; end // donor index 292
	10'd193: begin reg_addr=16'h501d; reg_data=8'h40; end // donor index 293
	10'd194: begin reg_addr=16'h5300; reg_data=8'h08; end // donor index 294
	10'd195: begin reg_addr=16'h5301; reg_data=8'h30; end // donor index 295
	10'd196: begin reg_addr=16'h5302; reg_data=8'h10; end // donor index 296
	10'd197: begin reg_addr=16'h5303; reg_data=8'h00; end // donor index 297
	10'd198: begin reg_addr=16'h5304; reg_data=8'h08; end // donor index 298
	10'd199: begin reg_addr=16'h5305; reg_data=8'h30; end // donor index 299
	10'd200: begin reg_addr=16'h5306; reg_data=8'h08; end // donor index 300
	10'd201: begin reg_addr=16'h5307; reg_data=8'h16; end // donor index 301
	10'd202: begin reg_addr=16'h5309; reg_data=8'h08; end // donor index 302
	10'd203: begin reg_addr=16'h530a; reg_data=8'h30; end // donor index 303
	10'd204: begin reg_addr=16'h530b; reg_data=8'h04; end // donor index 304
	10'd205: begin reg_addr=16'h530c; reg_data=8'h06; end // donor index 305
	10'd206: begin reg_addr=16'h5025; reg_data=8'h00; end // donor index 306
	10'd207: begin reg_addr=16'h3008; reg_data=8'h02; end // donor index 307
	10'd208: begin reg_addr=16'h3035; reg_data=8'h21; end // donor index 308
	10'd209: begin reg_addr=16'h3036; reg_data=8'h69; end // donor index 309
	10'd210: begin reg_addr=16'h3c07; reg_data=8'h08; end // donor index 310
	10'd211: begin reg_addr=16'h3820; reg_data=8'h47; end // donor index 311
	10'd212: begin reg_addr=16'h3821; reg_data=8'h00; end // donor index 312
	10'd213: begin reg_addr=16'h3814; reg_data=8'h31; end // donor index 313
	10'd214: begin reg_addr=16'h3815; reg_data=8'h31; end // donor index 314
	10'd215: begin reg_addr=16'h3800; reg_data=8'h00; end // donor index 315
	10'd216: begin reg_addr=16'h3801; reg_data=8'h00; end // donor index 316
	10'd217: begin reg_addr=16'h3802; reg_data=8'h00; end // donor index 317
	10'd218: begin reg_addr=16'h3803; reg_data=8'hfa; end // donor index 318
	10'd219: begin reg_addr=16'h3804; reg_data=8'h0a; end // donor index 319
	10'd220: begin reg_addr=16'h3805; reg_data=8'h3f; end // donor index 320
	10'd221: begin reg_addr=16'h3806; reg_data=8'h06; end // donor index 321
	10'd222: begin reg_addr=16'h3807; reg_data=8'ha9; end // donor index 322
	10'd223: begin reg_addr=16'h3808; reg_data=8'h05; end // donor index 323
	10'd224: begin reg_addr=16'h3809; reg_data=8'h00; end // donor index 324
	10'd225: begin reg_addr=16'h380a; reg_data=8'h02; end // donor index 325
	10'd226: begin reg_addr=16'h380b; reg_data=8'hd0; end // donor index 326
	10'd227: begin reg_addr=16'h380c; reg_data=8'h07; end // donor index 327
	10'd228: begin reg_addr=16'h380d; reg_data=8'h64; end // donor index 328
	10'd229: begin reg_addr=16'h380e; reg_data=8'h02; end // donor index 329
	10'd230: begin reg_addr=16'h380f; reg_data=8'he4; end // donor index 330
	10'd231: begin reg_addr=16'h3813; reg_data=8'h04; end // donor index 331
	10'd232: begin reg_addr=16'h3618; reg_data=8'h00; end // donor index 332
	10'd233: begin reg_addr=16'h3612; reg_data=8'h29; end // donor index 333
	10'd234: begin reg_addr=16'h3709; reg_data=8'h52; end // donor index 334
	10'd235: begin reg_addr=16'h370c; reg_data=8'h03; end // donor index 335
	10'd236: begin reg_addr=16'h3a02; reg_data=8'h02; end // donor index 336
	10'd237: begin reg_addr=16'h3a03; reg_data=8'he0; end // donor index 337
	10'd238: begin reg_addr=16'h3a08; reg_data=8'h00; end // donor index 338
	10'd239: begin reg_addr=16'h3a09; reg_data=8'h6f; end // donor index 339
	10'd240: begin reg_addr=16'h3a0a; reg_data=8'h00; end // donor index 340
	10'd241: begin reg_addr=16'h3a0b; reg_data=8'h5c; end // donor index 341
	10'd242: begin reg_addr=16'h3a0e; reg_data=8'h06; end // donor index 342
	10'd243: begin reg_addr=16'h3a0d; reg_data=8'h08; end // donor index 343
	10'd244: begin reg_addr=16'h3a14; reg_data=8'h02; end // donor index 344
	10'd245: begin reg_addr=16'h3a15; reg_data=8'he0; end // donor index 345
	10'd246: begin reg_addr=16'h4004; reg_data=8'h02; end // donor index 346
	10'd247: begin reg_addr=16'h3002; reg_data=8'h1c; end // donor index 347
	10'd248: begin reg_addr=16'h3006; reg_data=8'hc3; end // donor index 348
	10'd249: begin reg_addr=16'h4713; reg_data=8'h03; end // donor index 349
	10'd250: begin reg_addr=16'h4407; reg_data=8'h04; end // donor index 350
	10'd251: begin reg_addr=16'h460b; reg_data=8'h37; end // donor index 351
	10'd252: begin reg_addr=16'h460c; reg_data=8'h20; end // donor index 352
	10'd253: begin reg_addr=16'h4837; reg_data=8'h16; end // donor index 353
	10'd254: begin reg_addr=16'h3824; reg_data=8'h04; end // donor index 354
	10'd255: begin reg_addr=16'h5001; reg_data=8'h83; end // donor index 355
	10'd256: begin reg_addr=16'h3503; reg_data=8'h00; end // donor index 356
