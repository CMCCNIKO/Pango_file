# 最终布局布线资源报告摘录

以下摘自 Fabric Compiler `2025.2`（build `211867`）各自 `run_route_optimize.log` 的 `Device Utilization Summary`。列间空格已规范化，仅保留 README 引用的条目，未包含构建机信息。

## PG2L100H V030

- 器件：`PG2L100H-6FBG676`；顶层：`top_design`
- 构建：`v030_isp_dehaze_continuity/PG2L100H_final`
- 日志头时间：`Mon Nov 16 22:01:05 2026`
- 工程目录内原始日志：`prj_tasks/pnr_1/place_route/run_route_optimize.log`，资源段从第 92 行开始
- 原始日志 SHA-256：`BC34668DD7543EB7587B4506A2085647ED28C72B0C0AC5B7E43F60DE8565DA7D`

| Device Type | Used | Available | Util(%) |
| --- | ---: | ---: | ---: |
| CLMA | 11586 | 11675 | 100 |
| + FF | 58418 | 93400 | 63 |
| + LUT | 42395 | 46700 | 91 |
| CLMS | 4937 | 4975 | 100 |
| + FF | 23963 | 39800 | 61 |
| + LUT | 18082 | 19900 | 91 |
| IO | 180 | 300 | 60 |
| APM(grid) | 22 | 120 | 19 |
| + APM | 40 | 240 | 17 |
| BKCL | 6 | 6 | 100 |
| DDR_PHY | 7 | 24 | 30 |
| DRM(grid) | 144 | 155 | 93 |
| + DRM18K | 54 | 310 | 18 |
| + DRM36K | 112 | 155 | 73 |
| GPLL | 3 | 6 | 50 |
| HSSTLP | 2 | 2 | 100 |
| + HSSTLP_LANE | 5 | 8 | 63 |
| PCIE | 1 | 1 | 100 |
| PPLL | 2 | 6 | 34 |

日志随后记录 `Design 'top_design' has been routed successfully.`。

## PG2L200H V011

- 器件：`PG2L200H-6FBB484`；顶层：`host_hdmi_to_fiber_top`
- 构建：`v011_pg2l200h_pds2025_no_hdmi_out/PG2L200H_CNN_baseline`
- 日志头时间：`Sun Nov 15 20:18:23 2026`
- 工程目录内原始日志：`prj_tasks/pnr_1/place_route/run_route_optimize.log`，资源段从第 78 行开始
- 原始日志 SHA-256：`3FABA97B4DA1E3AAC7FF6A358B0089733CFB9E52A67352BF7051BFBC51FFB86F`

| Device Type | Used | Available | Util(%) |
| --- | ---: | ---: | ---: |
| CLMA | 15064 | 30075 | 51 |
| + FF | 49388 | 240600 | 21 |
| + LUT | 48235 | 120300 | 41 |
| CLMS | 4202 | 9875 | 43 |
| + FF | 15668 | 79000 | 20 |
| + LUT | 13986 | 39500 | 36 |
| IO | 52 | 285 | 19 |
| APM(grid) | 178 | 370 | 49 |
| + APM | 350 | 740 | 48 |
| BKCL | 4 | 10 | 40 |
| DRM(grid) | 386 | 415 | 94 |
| + DRM18K | 2 | 830 | 1 |
| + DRM36K | 385 | 415 | 93 |
| GPLL | 1 | 10 | 10 |
| HSSTLP | 1 | 1 | 100 |
| + HSSTLP_LANE | 1 | 4 | 25 |

日志随后记录 `Design 'host_hdmi_to_fiber_top' has been routed successfully.`。
