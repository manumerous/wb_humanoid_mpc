"""****************************************************************************
Copyright (c) 2024, 1X Technologies. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
****************************************************************************"""

import pandas as pd
import vpselector


def main():
    plot_config_dict = {
        "x_axis_col": "time",
        "sub_plt1_data": [
            "h_x",
            "h_y",
            "h_z",
            "L_x",
            "L_y",
            "L_z",
        ],
        "sub_plt2_data": [
            "p_x",
            "p_y",
            "p_z",
            "euler_z",
            "euler_y",
            "euler_x",
        ],
        "sub_plt3_data": [
            "q_j_l_hip_y",
            "q_j_l_hip_x",
            "q_j_l_hip_z",
            "q_j_l_upperknee_y",
            "q_j_l_lowerknee_y",
            "q_j_l_ankle_y",
            "q_j_l_ankle_x",
        ],
        "sub_plt4_data": [
            "q_j_r_hip_y",
            "q_j_r_hip_x",
            "q_j_r_hip_z",
            "q_j_r_upperknee_y",
            "q_j_r_lowerknee_y",
            "q_j_r_ankle_y",
            "q_j_r_ankle_x",
        ],
    }
    log_name = "mpc_observation_20240701_093238"
    data_df = pd.read_csv(log_name + ".csv")
    print(data_df.columns)
    print(data_df)

    selected_df = vpselector.data_selection.select_visual_data(
        data_df, plot_config_dict
    )
    print("Selected dataframe:")
    selected_df.self.data_df.to_csv(log_name + "_cropped.csv", index=False)


if __name__ == "__main__":
    main()
