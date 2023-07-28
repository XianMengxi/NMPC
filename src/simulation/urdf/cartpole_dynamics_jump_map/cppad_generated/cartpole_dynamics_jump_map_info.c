void cartpole_dynamics_jump_map_info(const char** baseName,
                                     unsigned long* m,
                                     unsigned long* n,
                                     unsigned int* indCount,
                                     unsigned int* depCount) {
   *baseName = "double  d";
   *m = 8;
   *n = 9;
   *depCount = 1; // number of dependent array variables
   *indCount = 1; // number of independent array variables
}

