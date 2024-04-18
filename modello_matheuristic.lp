\ Model Local_DASP
\ LP format - for model browsing. Use MPS format to capture full model detail.
Minimize
  tau_tilde + 0 x_k_ij[1,1,1] + 0 x_k_ij[1,2,2] + 0 x_k_ij[1,3,3]
   + 0 x_k_ij[1,4,4] + 0 x_k_ij[1,5,5] + 0 a_ki[1,0] + 0 a_ki[1,1]
   + 0 a_ki[1,2] + 0 a_ki[1,3] + 0 a_ki[1,4]
Subject To
 (16)[1]: - tau_tilde + a_ki[1,6] <= 0
 (17)[1,1]: - tau_tilde + a_ki[1,5] + 124.6885007617873 y_d_j[1,1]
   + 116.7737603706891 y_d_j[1,2] + 66.37435917246626 y_d_j[1,3]
   + 56.0257877132387 y_d_j[1,4] <= 0
 (17)[1,2]: - tau_tilde + a_ki[1,5] + 124.6885007617873 y_d_j[2,1]
   + 116.7737603706891 y_d_j[2,2] + 66.37435917246626 y_d_j[2,3]
   + 56.0257877132387 y_d_j[2,4] <= 0
 (18)[1]: x_k_ij[1,0,1] + x_k_ij[1,2,1] + x_k_ij[1,3,1] + x_k_ij[1,4,1]
   + x_k_ij[1,5,1] + y_d_j[1,1] + y_d_j[2,1] = 1
 (18)[2]: x_k_ij[1,0,2] + x_k_ij[1,1,2] + x_k_ij[1,3,2] + x_k_ij[1,4,2]
   + x_k_ij[1,5,2] + y_d_j[1,2] + y_d_j[2,2] = 1
 (18)[3]: x_k_ij[1,0,3] + x_k_ij[1,1,3] + x_k_ij[1,2,3] + x_k_ij[1,4,3]
   + x_k_ij[1,5,3] + y_d_j[1,3] + y_d_j[2,3] = 1
 (18)[4]: x_k_ij[1,0,4] + x_k_ij[1,1,4] + x_k_ij[1,2,4] + x_k_ij[1,3,4]
   + x_k_ij[1,5,4] + y_d_j[1,4] + y_d_j[2,4] = 1
 (19.1)[1]: x_k_ij[1,0,1] + x_k_ij[1,0,2] + x_k_ij[1,0,3] + x_k_ij[1,0,4]
   + x_k_ij[1,0,5] + x_k_ij[1,0,6] = 1
 (19.2)[1]: x_k_ij[1,0,6] + x_k_ij[1,1,6] + x_k_ij[1,2,6] + x_k_ij[1,3,6]
   + x_k_ij[1,4,6] + x_k_ij[1,5,6] = 1
 (20.1): x_k_ij[1,0,5] + x_k_ij[1,1,5] + x_k_ij[1,2,5] + x_k_ij[1,3,5]
   + x_k_ij[1,4,5] = 1
 (20.2): x_k_ij[1,5,1] + x_k_ij[1,5,2] + x_k_ij[1,5,3] + x_k_ij[1,5,4]
   + x_k_ij[1,5,6] = 1
Bounds
Binaries
 x_k_ij[1,0,1] x_k_ij[1,0,2] x_k_ij[1,0,3] x_k_ij[1,0,4] x_k_ij[1,0,5]
 x_k_ij[1,0,6] x_k_ij[1,1,1] x_k_ij[1,1,2] x_k_ij[1,1,3] x_k_ij[1,1,4]
 x_k_ij[1,1,5] x_k_ij[1,1,6] x_k_ij[1,2,1] x_k_ij[1,2,2] x_k_ij[1,2,3]
 x_k_ij[1,2,4] x_k_ij[1,2,5] x_k_ij[1,2,6] x_k_ij[1,3,1] x_k_ij[1,3,2]
 x_k_ij[1,3,3] x_k_ij[1,3,4] x_k_ij[1,3,5] x_k_ij[1,3,6] x_k_ij[1,4,1]
 x_k_ij[1,4,2] x_k_ij[1,4,3] x_k_ij[1,4,4] x_k_ij[1,4,5] x_k_ij[1,4,6]
 x_k_ij[1,5,1] x_k_ij[1,5,2] x_k_ij[1,5,3] x_k_ij[1,5,4] x_k_ij[1,5,5]
 x_k_ij[1,5,6] y_d_j[1,1] y_d_j[1,2] y_d_j[1,3] y_d_j[1,4] y_d_j[2,1]
 y_d_j[2,2] y_d_j[2,3] y_d_j[2,4]
End
