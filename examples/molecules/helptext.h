#define HELP_TEXT \
"Molecules\n(c) Matteo Golin 2026\n\nDESCRIPTION:\n    Ad-hoc aerial network " \
"formation inspired by molecules.\n\nUSAGE:\n    molecules [OPTIONS] n m\n\nO" \
"PTIONS:\n    -h          Display this help text.\n    -x <width>  Window wid" \
"th in pixels. Default is half screen width.\n    -y <height> Window height i" \
"n pixels. Default is half screen height.\n    -s <scale>  Rendering scale. D" \
"efault 5.\n    -d <limit>  Distance loss limit for severed connection in met" \
"ers. Default\n                50.0 m.\n    -z <height> Fixed z-coordinate (a" \
"ltitude) of UAVs. Default 10.0 m.\n    -r <seed>   Random seed to seed the r" \
"andom library with.\n    -m          If passed, ground units move randomly i" \
"nstead of being static.\n\nARGUMENTS:\n     n          Number of UAVs.\n    " \
" m          Number of ground units.\n\nCONTROLS:\n    This game is visualize" \
"d using SDL2 and accepts keyboard input.\n\n    q           Quit the game.\n" \
"    r           Toggle network connection radii.\n    n           Toggle net" \
"work connection lines.\n    Space       Reset the game.\n    Esc         Qui" \
"t the game.\n"
