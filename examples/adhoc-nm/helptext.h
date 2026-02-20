#define HELP_TEXT \
"Ad-hoc Network n > 0, m > 0\n\nDESCRIPTION:\n    This game is based on my pe" \
"rsonal formulation of a problem concerning the\n    optimal formation of an " \
"ad-hoc aerial network that provides connection to\n    independently moving " \
"evaders.\n\n    Pursuers (red) are given a random initial position within th" \
"e 3D simulation\n    space. Evaders (green) are given a random initial posit" \
"ion on the XY plane\n    at z = 0 (ground plane). Both agent types have simp" \
"le motion (holonomic).\n    The goal of the pursuers is to minimize path-los" \
"s of an RF signal\n    transmitted from the pursuers to the evaders (and fro" \
"m pursuers to other\n    pursuers), while the evaders aim to maximize the pa" \
"th-loss.\n\nUSAGE:\n    adhoc-nm [OPTIONS]\n\nOPTIONS:\n    -h          Disp" \
"lay this help text.\n    -x <width>  Window/simulation width in pixels. Defa" \
"ult 1/2 screen width.\n    -y <height> Window/simulation height in pixels. D" \
"efault 1/2 screen height.\n    -z <height> Height of the z-dimension in the " \
"simulation. Default 100.\n    -s <scale>  Rendering scale. Default 5.\n    -" \
"l <loss>   Maximum allowable loss for a connection in dB. Default 45.0.\n   " \
" -m <m>      Number of evaders m > 0. Default 1.\n    -n <n>      Number of " \
"evaders n > 0. Default 1.\n\nCONTROLS:\n    This game is visualized using SD" \
"L2 and accepts keyboard input.\n\n    q           Quit the game.\n    Esc   " \
"      Quit the game.\n    Space       Re-seed and re-start the game.\n    n " \
"          Render the network connections.\n"
