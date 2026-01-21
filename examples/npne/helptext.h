#define HELP_TEXT \
"N Pursuers, M Evaders\n\nDESCRIPTION:\n    This game is based on the paper " \
"Multiple Pursuers Multiple Evader\n    Differential Games\" by Eloy Garcia, " \
"David W. Casbeer, Alexander Von Moll and\n    Meir Pachter.\n\n    The game " \
"consists of N pursuers and M evaders, where N = M. The goal of the\n    purs" \
"uers is to capture the evaders (come within some capture radius distance\n  " \
"  of the evaders) in the shortest time possible. The evaders aim to avoid\n " \
"   capture for as long as possible.\n\n    Pursuers are always faster than e" \
"vaders. All agents have holonomic motion in\n    an infinite 2D plane. The p" \
"layers are controlled by their optimal control\n    signals from Section IV-" \
"A of the paper. At run-time, the players are all\n    assigned random initia" \
"l conditions (start locations and headings), as well\n    as a random veloci" \
"ty within some allowable range. Pursuer velocities are\n    within [30, 40] " \
"and evader velocities are within [10, 29].\n\nUSAGE:\n    npme [OPTIONS]\n\n" \
"OPTIONS:\n    -h          Display this help text.\n    -x <width>  Window wi" \
"dth in pixels. Default is half screen width.\n    -y <height> Window height " \
"in pixels. Default is half screen height.\n    -s <scale>  Rendering scale. " \
"Default 5.\n    -r <radius> Capture radius of the pursuers in meters. Defaul" \
"t 0.\n    -n <num>    Number of pursuers and evaders. Default 2.\n\nCONTROLS" \
":\n    This game is visualized using SDL2 and accepts keyboard input.\n\n   " \
" q           Quit the game.\n    Esc         Quit the game.\n    r          " \
" Toggle visualization of the pursuer capture radius.\n    Space       Re-see" \
"d and re-start the game.\n"
