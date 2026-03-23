#define HELP_TEXT \
"Replayed Ad-hoc Aerial Game\n\nDESCRIPTION:\n    Replay the simulation of an" \
" ad-hoc aerial game from a file which records\n    positions over time.\n\n " \
"   File records should be in the format of:\n\n    n,m\n    0,x,y,z\n    1,x" \
",y,z\n    ...\n\n    Where 'n' is the number of pursuers and 'm' is the numb" \
"er of evaders. Each\n    record begins with the agent's ID followed by their" \
" X, Y, Z position as\n    floating point numbers. IDs should be < n for purs" \
"uers and >= n for evaders.\n\nUSAGE:\n    replay [OPTIONS]\n\nOPTIONS:\n    " \
"-h          Display this help text.\n    -x <width>  Window width in pixels." \
" Default is half screen width.\n    -y <height> Window height in pixels. Def" \
"ault is half screen height.\n    -s <scale>  Rendering scale. Default 5.\n  " \
"  -f <file>   File to replay from. Required.\n    -t <step>   Time step (fra" \
"me) duration in microseconds. Default 0.\n    -e          When this flag is " \
"passed, the program will exit on game over.\n    -l <limit>  Path loss limit" \
" for severed connection in dB. Overrides the\n                value in the s" \
"imulation file.\n\nCONTROLS:\n    This game is visualized using SDL2 and acc" \
"epts keyboard input.\n\n    q           Quit the game.\n    n           Show" \
" network connections.\n    Esc         Quit the game.\n    Space       Re-st" \
"art the game.\n"
