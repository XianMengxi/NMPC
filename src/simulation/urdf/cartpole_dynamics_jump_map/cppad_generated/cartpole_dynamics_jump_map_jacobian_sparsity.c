void cartpole_dynamics_jump_map_jacobian_sparsity(unsigned long const** row,
                                                  unsigned long const** col,
                                                  unsigned long* nnz) {
   static unsigned long const rows[8] = {0,1,2,3,4,5,6,7};
   static unsigned long const cols[8] = {1,2,3,4,5,6,7,8};
   *row = rows;
   *col = cols;
   *nnz = 8;
}
