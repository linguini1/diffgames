#define HELP_TEXT \
"2 Pursuers, 2 Evaders\n\nDESCRIPTION:\n    This game is based on the paper " \
"Multiple Pursuers Multiple Evader\n    Differential Games\" by Eloy Garcia, " \
"David W. Casbeer, Alexander Von Moll and\n    Meir Pachter.\n\n    The game " \
"consists of two pursuers and two evaders. The goal of the pursuers\n    is t" \
"o capture the evaders (come within some capture radius distance of the\n    " \
"evaders) in the shortest time possible. The evaders aim to avoid capture for" \
"\n    as long as possible.\n\n    The two pursuers are always faster than bo" \
"th of the evaders. All agents have\n    holonomic motion in an infinite 2D p" \
"lane. The players are controlled by\n    their optimal control signals from " \
"Section III of the paper. At run-time,\n    the players are all assigned ran" \
"dom initial conditions (start locations and\n    headings).\n\n    The game " \
"ends when an evader is captured.\n\nUSAGE:\n    2p2e [OPTIONS]\n\nOPTIONS:\n" \
"    -h          Display this help text.\n    -x <width>  Window width in pix" \
"els. Default 1024.\n    -y <height> Window height in pixels. Default 1024.\n" \
"    -s <scale>  Rendering scale. Default 5.\n    -r <radius> Capture radius " \
"of the pursuers in meters. Default 0.\n\nCONTROLS:\n    This game is visuali" \
"zed using SDL2 and accepts keyboard input.\n\n    q           Quit the game." \
"\n    Esc         Quit the game.\n    r           Toggle visualization of th" \
"e pursuer capture radius.\n    Space       Re-seed and re-start the game.\n"
