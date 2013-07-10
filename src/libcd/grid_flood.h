/** \file grid_flood.h
 * \brief Interface to cd_grid_flood, a flood-fill implementation.
 * \author Christopher Dellin
 * \date 2011-2012
 */

/* (C) Copyright 2011-2013 Carnegie Mellon University */

/* This module (cd_grid_flood) is part of libcd.
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
 * #include <stdlib.h>
 * #include <libcd/grid.h>
 */

/* Modify the grid by performing a flood-fill starting with the given index.
 * Diagonal cells are considered adjacent (e.g. 8-connected in 2D).
 * By default, the grid does not wrap; if wrap_dim is passed, it should
 * have length grid.n, with non-zero elements denoting a wrapped dimension. */
int cd_grid_flood_fill(struct cd_grid * g, size_t index_start,
   int * wrap_dim,
   int (*replace)(void *, void *), void * rptr);
