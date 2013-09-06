#include <hubo.h>
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <math.h>

int main(int argc, char** argv){

  
  ach_channel_t chan_hubo_ref;  
  ach_channel_t chan_hubo_state;

  /* open the reference channel - command sent to the robot */
  ach_status_t r;

  r = ach_open(&chan_hubo_ref, HUBO_CHAN_REF_NAME , NULL);
  if (r != ACH_OK) {
    fprintf(stderr, "error opening hubo ref channel: %s\n",   ach_result_to_string(r));
    exit(1);
  }

  hubo_ref_t H_ref;
  hubo_state_t H_state;
  size_t fs;


  /* get current reference */
  int curtry;
  const int ntries = 3;


  for (curtry=0; curtry<ntries; ++curtry) {
    r = ach_get( &chan_hubo_ref, &H_ref, sizeof(H_ref), &fs, NULL, ACH_O_LAST );
    if (r == ACH_OK || r == ACH_STALE_FRAMES) {
      break; 
    } else {
      if (!(curtry == 0 && r == ACH_MISSED_FRAME)) {
        fprintf(stderr, "error getting init ref on curtry %d/%d: %s\n",
                curtry+1, ntries,ach_result_to_string(r));
      }
    }
  }

  if (r != ACH_OK && r != ACH_STALE_FRAMES) {
    exit(1);
  } else {
    assert(fs == sizeof(H_ref));
  }

// Part 2
r = ach_open(&chan_hubo_state, HUBO_CHAN_STATE_NAME, NULL);
  if (r != ACH_OK) {
    fprintf(stderr, "error opening hubo state channel: %s\n", ach_result_to_string(r));
    exit(1);
  }

  for (curtry=0; curtry<ntries; ++curtry) {
    r = ach_get( &chan_hubo_state, &H_state, sizeof(H_state), &fs, NULL, ACH_O_WAIT );
    if (r == ACH_OK) { 
      break; 
    } else {
      if (!(curtry == 0 && r == ACH_MISSED_FRAME)) {
        fprintf(stderr, "error getting init state on curtry %d/%d: %s\n",
                curtry+1, ntries, ach_result_to_string(r));
      }
    }
  }

  if (r != ACH_OK) {
    exit(1);
  } else {
    assert(fs == sizeof(H_state));
  }

  r = ach_flush(&chan_hubo_state);
  if (r != ACH_OK) {
    fprintf(stderr, "error flushing state channel: %s\n", 
            ach_result_to_string(r));
  }

// move elbow with sine wave
  double period = 2.0; // sec
  FILE* outputfile = fopen("output.txt","w");

  for(double t=0; t<4*period; t+=HUBO_LOOP_PERIOD){
  // first get state by waiting
  // From hubomz..
  r = ach_get( &chan_hubo_state, &H_state, sizeof(H_state), &fs, NULL, ACH_O_WAIT );
      if (r != ACH_OK) {
        fprintf(stderr, "warning: getting hubo state: %s\n", ach_result_to_string(r));
      } else {
        assert(fs == sizeof(H_state));
      }

  double lf_mx = H_state.ft[HUBO_FT_L_FOOT].m_x;
  double lf_my = H_state.ft[HUBO_FT_L_FOOT].m_y;
  double lf_fz = H_state.ft[HUBO_FT_L_FOOT].f_z;

  double lsp_encoder = H_state.joint[LSP].pos;


// next set ref with angle
    double lsp_angle = sin(t*2*M_PI/period)*5*M_PI/180;
    H_ref.ref[LSP] = lsp_angle;
    H_ref.mode[LSP] = 1; // unfiltered, 0 = filter (check in hubo.h)
    
    r = ach_put( &chan_hubo_ref, &H_ref, sizeof(H_ref) );                  
      if (r != ACH_OK) {
        fprintf(stderr, "warning: putting hubo ref: %s\n", ach_result_to_string(r));
      }

  fprintf(outputfile,"%f %f %f %f\n",lf_mx,lf_my,lf_fz,lsp_encoder);
    
  }

	fclose(outputfile);
	return 0;
}
