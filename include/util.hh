
#include "BitVector.hh"
#include "H264or5VideoStreamFramer.hh"

#define VPS_MAX_SIZE \
  1024  // larger than the largest possible VPS (Video Parameter Set) NAL unit
#define SPS_MAX_SIZE \
  1024  // larger than the largest possible SPS (Sequence Parameter Set) NAL
        // unit

void profile_tier_level(BitVector& bv, unsigned max_sub_layers_minus1) {
  bv.skipBits(96);

  unsigned i;
  Boolean sub_layer_profile_present_flag[7], sub_layer_level_present_flag[7];
  for (i = 0; i < max_sub_layers_minus1; ++i) {
    sub_layer_profile_present_flag[i] = bv.get1BitBoolean();
    sub_layer_level_present_flag[i] = bv.get1BitBoolean();
  }
  if (max_sub_layers_minus1 > 0) {
    bv.skipBits(2 * (8 - max_sub_layers_minus1));  // reserved_zero_2bits
  }
  for (i = 0; i < max_sub_layers_minus1; ++i) {
    if (sub_layer_profile_present_flag[i]) {
      bv.skipBits(88);
    }
    if (sub_layer_level_present_flag[i]) {
      bv.skipBits(8);  // sub_layer_level_idc[i]
    }
  }
}

void analyze_video_parameter_set(char* data, unsigned size,
                                 unsigned& num_units_in_tick,
                                 unsigned& time_scale) {
  num_units_in_tick = time_scale = 0;  // default values
  BitVector bv(vps, 0, 8 * size);

  // Begin by making a copy of the NAL unit data, removing any 'emulation
  // prevention' bytes:
  u_int8_t vps[VPS_MAX_SIZE];
  unsigned vpsSize;
  vpsSize = removeH264or5EmulationBytes(vps, VPS_MAX_SIZE, data, size);

  BitVector bv(vps, 0, 8 * vpsSize);

  unsigned i;

  bv.skipBits(28);  // nal_unit_header, vps_video_parameter_set_id,
                    // vps_reserved_three_2bits, vps_max_layers_minus1
  unsigned vps_max_sub_layers_minus1 = bv.getBits(3);
  bv.skipBits(17);  // vps_temporal_id_nesting_flag, vps_reserved_0xffff_16bits
  profile_tier_level(bv, vps_max_sub_layers_minus1);
  Boolean vps_sub_layer_ordering_info_present_flag = bv.get1BitBoolean();
  for (i = vps_sub_layer_ordering_info_present_flag ? 0
                                                    : vps_max_sub_layers_minus1;
       i <= vps_max_sub_layers_minus1; ++i) {
    (void)bv.get_expGolomb();  // vps_max_dec_pic_buffering_minus1[i]
    (void)bv.get_expGolomb();  // vps_max_num_reorder_pics[i]
    (void)bv.get_expGolomb();  // vps_max_latency_increase_plus1[i]
  }
  unsigned vps_max_layer_id = bv.getBits(6);
  unsigned vps_num_layer_sets_minus1 = bv.get_expGolomb();
  for (i = 1; i <= vps_num_layer_sets_minus1; ++i) {
    bv.skipBits(vps_max_layer_id +
                1);  // layer_id_included_flag[i][0..vps_max_layer_id]
  }
  Boolean vps_timing_info_present_flag = bv.get1BitBoolean();
  if (vps_timing_info_present_flag) {
    num_units_in_tick = bv.getBits(32);
    time_scale = bv.getBits(32);
    Boolean vps_poc_proportional_to_timing_flag = bv.get1BitBoolean();
    if (vps_poc_proportional_to_timing_flag) {
      unsigned vps_num_ticks_poc_diff_one_minus1 = bv.get_expGolomb();
    }
  }
  Boolean vps_extension_flag = bv.get1BitBoolean();
}

void analyze_seq_parameter_set_data(
    char* data, unsigned size, unsigned pic_width_in_luma_samples,
    unsigned& pic_height_in_luma_samples unsigned& num_units_in_tick,
    unsigned& time_scale) {
  unsigned pic_width_in_luma_samples, pic_height_in_luma_samples,
      log2_max_pic_order_cnt_lsb_minus4;
  num_units_in_tick = time_scale = 0;  // default values

  // Begin by making a copy of the NAL unit data, removing any 'emulation
  // prevention' bytes:
  u_int8_t sps[SPS_MAX_SIZE];
  unsigned spsSize;
  removeEmulationBytes(sps, sizeof sps, spsSize);
  spsSize = removeH264or5EmulationBytes(sps, VPS_MAX_SIZE, data, size);

  BitVector bv(sps, 0, 8 * spsSize);
  {  // 265
    unsigned i;

    bv.skipBits(16);  // nal_unit_header
    bv.skipBits(4);   // sps_video_parameter_set_id
    unsigned sps_max_sub_layers_minus1 = bv.getBits(3);
    bv.skipBits(1);  // sps_temporal_id_nesting_flag
    profile_tier_level(bv, sps_max_sub_layers_minus1);
    (void)bv.get_expGolomb();  // sps_seq_parameter_set_id
    unsigned chroma_format_idc = bv.get_expGolomb();
    if (chroma_format_idc == 3) bv.skipBits(1);  // separate_colour_plane_flag
    unsigned pic_width_in_luma_samples = bv.get_expGolomb();
    unsigned pic_height_in_luma_samples = bv.get_expGolomb();
    Boolean conformance_window_flag = bv.get1BitBoolean();
    if (conformance_window_flag) {
      unsigned conf_win_left_offset = bv.get_expGolomb();
      unsigned conf_win_right_offset = bv.get_expGolomb();
      unsigned conf_win_top_offset = bv.get_expGolomb();
      unsigned conf_win_bottom_offset = bv.get_expGolomb();
    }
    (void)bv.get_expGolomb();  // bit_depth_luma_minus8
    (void)bv.get_expGolomb();  // bit_depth_chroma_minus8
    unsigned log2_max_pic_order_cnt_lsb_minus4 = bv.get_expGolomb();
    Boolean sps_sub_layer_ordering_info_present_flag = bv.get1BitBoolean();
    for (i = (sps_sub_layer_ordering_info_present_flag
                  ? 0
                  : sps_max_sub_layers_minus1);
         i <= sps_max_sub_layers_minus1; ++i) {
      (void)bv.get_expGolomb();  // sps_max_dec_pic_buffering_minus1[i]
      (void)bv.get_expGolomb();  // sps_max_num_reorder_pics[i]
      (void)bv.get_expGolomb();  // sps_max_latency_increase[i]
    }
    (void)bv.get_expGolomb();  // log2_min_luma_coding_block_size_minus3
    (void)bv.get_expGolomb();  // log2_diff_max_min_luma_coding_block_size
    (void)bv.get_expGolomb();  // log2_min_transform_block_size_minus2
    (void)bv.get_expGolomb();  // log2_diff_max_min_transform_block_size
    (void)bv.get_expGolomb();  // max_transform_hierarchy_depth_inter
    (void)bv.get_expGolomb();  // max_transform_hierarchy_depth_intra
    Boolean scaling_list_enabled_flag = bv.get1BitBoolean();
    if (scaling_list_enabled_flag) {
      Boolean sps_scaling_list_data_present_flag = bv.get1BitBoolean();
      if (sps_scaling_list_data_present_flag) {
        // scaling_list_data()
        for (unsigned sizeId = 0; sizeId < 4; ++sizeId) {
          for (unsigned matrixId = 0; matrixId < (sizeId == 3 ? 2 : 6);
               ++matrixId) {
            Boolean scaling_list_pred_mode_flag = bv.get1BitBoolean();
            if (!scaling_list_pred_mode_flag) {
              (void)bv
                  .get_expGolomb();  // scaling_list_pred_matrix_id_delta[sizeId][matrixId]
            } else {
              unsigned const c = 1 << (4 + (sizeId << 1));
              unsigned coefNum = c < 64 ? c : 64;
              if (sizeId > 1) {
                (void)bv
                    .get_expGolomb();  // scaling_list_dc_coef_minus8[sizeId][matrixId]
              }
              for (i = 0; i < coefNum; ++i) {
                (void)bv.get_expGolomb();  // scaling_list_delta_coef
              }
            }
          }
        }
      }
    }
    bv.skipBits(2);  // amp_enabled_flag, sample_adaptive_offset_enabled_flag
    Boolean pcm_enabled_flag = bv.get1BitBoolean();
    if (pcm_enabled_flag) {
      bv.skipBits(8);            // pcm_sample_bit_depth_luma_minus1,
                                 // pcm_sample_bit_depth_chroma_minus1
      (void)bv.get_expGolomb();  // log2_min_pcm_luma_coding_block_size_minus3
      (void)bv.get_expGolomb();  // log2_diff_max_min_pcm_luma_coding_block_size
      bv.skipBits(1);            // pcm_loop_filter_disabled_flag
    }
    unsigned num_short_term_ref_pic_sets = bv.get_expGolomb();
    unsigned num_negative_pics = 0, prev_num_negative_pics = 0;
    unsigned num_positive_pics = 0, prev_num_positive_pics = 0;
    for (i = 0; i < num_short_term_ref_pic_sets; ++i) {
      // short_term_ref_pic_set(i):
      Boolean inter_ref_pic_set_prediction_flag = False;
      if (i != 0) {
        inter_ref_pic_set_prediction_flag = bv.get1BitBoolean();
      }
      if (inter_ref_pic_set_prediction_flag) {
        if (i == num_short_term_ref_pic_sets) {
          // This can't happen here, but it's in the spec, so we include it for
          // completeness
          (void)bv.get_expGolomb();  // delta_idx_minus1
        }
        bv.skipBits(1);            // delta_rps_sign
        (void)bv.get_expGolomb();  // abs_delta_rps_minus1
        unsigned NumDeltaPocs =
            prev_num_negative_pics + prev_num_positive_pics;  // correct???
        for (unsigned j = 0; j < NumDeltaPocs; ++j) {
          Boolean used_by_curr_pic_flag = bv.get1BitBoolean();
          if (!used_by_curr_pic_flag) bv.skipBits(1);  // use_delta_flag[j]
        }
      } else {
        prev_num_negative_pics = num_negative_pics;
        num_negative_pics = bv.get_expGolomb();
        prev_num_positive_pics = num_positive_pics;
        num_positive_pics = bv.get_expGolomb();
        unsigned k;
        for (k = 0; k < num_negative_pics; ++k) {
          (void)bv.get_expGolomb();  // delta_poc_s0_minus1[k]
          bv.skipBits(1);            // used_by_curr_pic_s0_flag[k]
        }
        for (k = 0; k < num_positive_pics; ++k) {
          (void)bv.get_expGolomb();  // delta_poc_s1_minus1[k]
          bv.skipBits(1);            // used_by_curr_pic_s1_flag[k]
        }
      }
    }
    Boolean long_term_ref_pics_present_flag = bv.get1BitBoolean();
    if (long_term_ref_pics_present_flag) {
      unsigned num_long_term_ref_pics_sps = bv.get_expGolomb();
      for (i = 0; i < num_long_term_ref_pics_sps; ++i) {
        bv.skipBits(
            log2_max_pic_order_cnt_lsb_minus4);  // lt_ref_pic_poc_lsb_sps[i]
        bv.skipBits(1);  // used_by_curr_pic_lt_sps_flag[1]
      }
    }
    bv.skipBits(2);  // sps_temporal_mvp_enabled_flag,
                     // strong_intra_smoothing_enabled_flag
    Boolean vui_parameters_present_flag = bv.get1BitBoolean();
    if (vui_parameters_present_flag) {
      analyze_vui_parameters(bv, num_units_in_tick, time_scale);
    }
    Boolean sps_extension_flag = bv.get1BitBoolean();
  }
}