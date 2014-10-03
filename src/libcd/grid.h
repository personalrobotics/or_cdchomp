/** \file grid.h
 * \brief Interface to cd_grid, a multidimensional grid of values.
 * \author Christopher Dellin
 * \date 2011-2012
 */

/* (C) Copyright 2011-2013 Carnegie Mellon University */

/* This module (cd_grid) is part of libcd.
 *
 * This module of libcd is free software: you can redistribute it
 * and/or modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This module of libcd is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * A copy of the GNU General Public License is provided with libcd
 * (license-gpl.txt) and is also available at <http://www.gnu.org/licenses/>.
 */

/* requires:
 * #include <stdlib.h> (for size_t)
 */

struct cd_grid
{
   /* Dimensionality of space */
   int n;
   /* Grid parameters */
   int * sizes;
   size_t ncells;
   /* The actual data */
   int cell_size;
   char * data;
   /* Actual grid side lengths (1x1x1x... by default) */
   double * lengths;
};

int cd_grid_create(struct cd_grid ** gp, void * cell_init, int cell_size, int n, ...);
int cd_grid_create_sizearray(struct cd_grid ** gp, void * cell_init, int cell_size, int n, int * sizes);

/* note - this takes ownership of sizes, and will free it when the grid is destroyed! */
int cd_grid_create_sizeown(struct cd_grid ** gp, void * cell_init, int cell_size, int n, int * sizes);

int cd_grid_create_copy(struct cd_grid ** gp, struct cd_grid * gsrc);
int cd_grid_destroy(struct cd_grid * g);

/* convert between index and subs */
int cd_grid_index_to_subs(struct cd_grid * g, size_t index, int * subs);
int cd_grid_index_from_subs(struct cd_grid * g, size_t * index, int * subs);

/* Get center from index */
int cd_grid_center_index(struct cd_grid * g, size_t index, double * center);
   
/* Lookup cell index, subs from location
 * these will return 1 if it's out of range! */
int cd_grid_lookup_index(struct cd_grid * g, double * p, size_t * index);
int cd_grid_lookup_subs(struct cd_grid * g, double * p, int * subs);


/* Get cell from index, subs */
void * cd_grid_get_index(struct cd_grid * g, size_t index);
void * cd_grid_get_subs(struct cd_grid * g, int * subs);
void * cd_grid_get(struct cd_grid * g, ...);


/* Simple multilinear interpolation / discontinuous gradient
 * these will return 1 if the point is out of range! */
int cd_grid_double_interp(struct cd_grid * g, double * p, double * valuep);
int cd_grid_double_grad(struct cd_grid * g, double * p, double * grad);



/* Squared Euclidean Distance Transform
 * (see http://www.cs.cornell.edu/~dph/papers/dt.pdf)
 * Uses doubles!
 * Given a double grid funcg,
 * computes the sedt grid (same size),
 * with 0 -> 0, and + -> squared distance to smallest value */
int cd_grid_double_dt_sqeuc(struct cd_grid ** gp_dt, struct cd_grid * g_func);
/* legacy */
int cd_grid_double_sedt(struct cd_grid ** gp_dt, struct cd_grid * g_func);

/* signed euclidean distance transform, aka signed distance field
 * g_binobs is type(char) with 0 in free space, !=0 in obstacles
 * output is + inside obstacles, - in free space */
int cd_grid_double_dt_sgneuc(struct cd_grid ** gp_dt, struct cd_grid * g_binobs);
/* legacy (g_emp is HUGE_VAL in obstacles, and 0.0 in free space) */
int cd_grid_double_bin_sdf(struct cd_grid ** gp_dt, struct cd_grid * g_emp);
