#define HELP_TEXT \
"Network Connectivity Visualizer\n(c) Matteo Golin 2026\n\nDESCRIPTION:\n    " \
"Visualize the connectivity of an ad-hoc network.\n\nUSAGE:\n    netconn [OPT" \
"IONS] n m\n\nOPTIONS:\n    -h          Display this help text.\n    -x <widt" \
"h>  Window width in pixels. Default is half screen width.\n    -y <height> W" \
"indow height in pixels. Default is half screen height.\n    -s <scale>  Rend" \
"ering scale. Default 5.\n    -l <limit>  Path loss limit for severed connect" \
"ion in dB.\n    -z <height> Fixed z-coordinate (altitude) of UAVs. Default 1" \
"0.0 m.\n    -r <seed>   Random seed to seed the random library with.\n\nARGU" \
"MENTS:\n     n          Number of UAVs.\n     m          Number of ground un" \
"its.\n\nCONTROLS:\n    This game is visualized using SDL2 and accepts keyboa" \
"rd input.\n\n    q           Quit the game.\n    n           Show network co" \
"nnections.\n    h           Show heatmap.\n    w           Move selected UAV" \
" up.\n    a           Move selected UAV left.\n    s           Move selected" \
" UAV down.\n    d           Move selected UAV right.\n    c           Move U" \
"AVs to high scoring locations.\n    Up          Select next UAV in the list." \
"\n    Down        Select previous UAV in the list.\n    Esc         Quit the" \
" game.\n    Space       Re-start the game.\n\n    Mouse click: Move selected" \
" UAV to clicked point.\n"
