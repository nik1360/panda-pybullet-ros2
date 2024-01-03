float32 time            
float32[49] mass_matrix     # Colum major vectorization of the 7x7 mass matrix M(q)
float32[7] coriolis         # Coriolis and centripetal term C(q, \dot{q})\dot{q}
float32[7] gravity          # Gravity term G(q)