#include <getopt.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include <SDL2/SDL.h>

#include "3dtools.h"
#include "helptext.h"
#include "render.h"
#include "utils.h"

static const char window_name[] = "3D Test";

#define DANGLE (0.05)
#define SLENGTH (10.0)

#define array_len(arr) (sizeof(arr) / sizeof(arr[0]))

static vec3d_t CUBE[] = {
    VEC3D_SINIT(0, 0, 0),     VEC3D_SINIT(0, 0, 1.0),
    VEC3D_SINIT(0, 1.0, 0),   VEC3D_SINIT(0, 1.0, 1.0),
    VEC3D_SINIT(1.0, 0, 0),   VEC3D_SINIT(1.0, 0, 1.0),
    VEC3D_SINIT(1.0, 1.0, 0), VEC3D_SINIT(1.0, 1.0, 1.0),
};

int main(int argc, char **argv) {
  double scale = 5.0;
  camera3d_t camera = {
      .pos = VEC3D_SINIT(0.0, 15.0, 40.0),
      .ori = VEC3D_SINIT(-M_PI_4 / 2, 0, 0),
  };
  SDL_Event event;
  SDL_DisplayMode dm = {0};
  SDL_DisplayMode tempdm;
  bool running = true;
  double rot_angle = 0.0;

  int c;
  while ((c = getopt(argc, argv, ":hx:y:s:")) != -1) {
    switch (c) {
    case 'h':
      puts(HELP_TEXT);
      exit(EXIT_SUCCESS);
      break;
    case 'x':
      dm.w = strtoul(optarg, NULL, 10);
      break;
    case 'y':
      dm.h = strtoul(optarg, NULL, 10);
      break;
    case 's':
      scale = strtod(optarg, NULL);
      break;
      break;
    case '?':
      fprintf(stderr, "Unknown option -%c\n", optopt);
      exit(EXIT_FAILURE);
      break;
    }
  }

  /* Set up OpenGL parameters */

  SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);

  /* Start SDL */

  if (SDL_Init(SDL_INIT_VIDEO) < 0) {
    printf("Could not initialize SDL: %s\n", SDL_GetError());
  }

  SDL_GetDesktopDisplayMode(0, &tempdm);
  if (dm.w == 0) dm.w = tempdm.w / 2;
  if (dm.h == 0) dm.h = tempdm.h / 2;
  vec2d_t screen_center =
      VEC2D_SINIT(dm.w / (2.0 * scale), dm.h / (2.0 * scale));

  /* Create window */

  SDL_Window *window =
      SDL_CreateWindow(window_name, SDL_WINDOWPOS_UNDEFINED,
                       SDL_WINDOWPOS_UNDEFINED, dm.w, dm.h, SDL_WINDOW_OPENGL);

  /* Create renderer */

  SDL_Renderer *renderer = SDL_CreateRenderer(
      window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
  SDL_RenderSetScale(renderer, scale, scale);

  /* Scale cube */

  for (size_t i = 0; i < array_len(CUBE); i++) {
    vec3d_scale(&CUBE[i], SLENGTH, &CUBE[i]);
  }

  printf("Camera: " VEC3D_FMT "\n", VEC3D_PRINTF(&camera.pos));
  printf("CUBE[0]: " VEC3D_FMT "\n", VEC3D_PRINTF(&CUBE[0]));

  while (running) {

    /* Check for input events */

    while (SDL_PollEvent(&event)) {

      switch (event.type) {

      case SDL_QUIT:
        running = false;
        break;

      case SDL_KEYDOWN:
        switch (event.key.keysym.sym) {

        case SDLK_ESCAPE:
        case SDLK_q:
          running = false;
          break;

        default:
          break;
        }
        break;

      default:
        break;
      }
    }

    /* Clear screen to black with opacity so trail is visible */

    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, SDL_ALPHA_OPAQUE);
    SDL_RenderClear(renderer);

    /* Set to foreground colour (white) */

    SDL_SetRenderDrawColor(renderer, 255, 255, 255, SDL_ALPHA_OPAQUE);

    /* Draw object */

    vec3d_t vrot;
    vec2d_t proj;
    for (unsigned i = 0; i < sizeof(CUBE) / sizeof(CUBE[0]); i++) {

      /* Rotate each vector about the z_axis and then project it to 2D.
       * Projection is translated to screen center.
       */

      vec3d_rotate(&CUBE[i], rot_angle, AXIS_Y, &vrot);
      camera_proj(&vrot, &camera, &proj);
      vec2d_add(&proj, &screen_center, &proj);
      render_vec2d(renderer, &proj);
    }

    /* Show what was drawn */

    SDL_RenderPresent(renderer);

    /* Advance animation */

    rot_angle += DANGLE;
    if (rot_angle > 2 * M_PI) rot_angle = 0;
  }

  /* Release resources */

  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);
  SDL_Quit();

  return EXIT_SUCCESS;
}
