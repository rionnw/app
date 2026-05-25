#include "Cubie.h"

#include <algorithm>
#include <random>
#include "Coord.h"

namespace cubie {

	/* Faster than tricky if-else sequences for handling mirrored states */
	int mul_coris[][6] = {
	  {0, 1, 2, 3, 4, 5},
	  {1, 2, 0, 4, 5, 3},
	  {2, 0, 1, 5, 3, 4},
	  {3, 5, 4, 0, 2, 1},
	  {4, 3, 5, 1, 0, 2},
	  {5, 4, 3, 2, 1, 0}
	};
	int inv_cori[] = {
	  0, 2, 1, 3, 4, 5
	};

	std::random_device device;
	std::mt19937 gen(device());

	void corner::mul(const cubie::cube& c1, const cubie::cube& c2, cubie::cube& into) {
		for (int i = 0; i < corner::COUNT; i++) {
			into.cperm[i] = c1.cperm[c2.cperm[i]];
			into.cori[i] = mul_coris[c1.cori[c2.cperm[i]]][c2.cori[i]];
		}
	}

	void edge::mul(const cubie::cube& c1, const cubie::cube& c2, cubie::cube& into) {
		for (int i = 0; i < edge::COUNT; i++) {
			into.eperm[i] = c1.eperm[c2.eperm[i]];
			into.eori[i] = (c1.eori[c2.eperm[i]] + c2.eori[i]) & 1;
		}
	}

	// Permutation partiy =  #inversions % 2
	bool parity(const int perm[], int len) {
		int par = 0;
		for (int i = 0; i < len; i++) {
			for (int j = 0; j < i; j++) {
				if (perm[j] > perm[i])
					par++;
			}
		}
		return par & 1;
	}

	void mul(const cubie::cube& c1, const cubie::cube& c2, cubie::cube& into) {
		corner::mul(c1, c2, into);
		edge::mul(c1, c2, into);
	}

	void inv(const cube& c, cube& into) {
		for (int corner = 0; corner < corner::COUNT; corner++)
			into.cperm[c.cperm[corner]] = corner; // inv[a[i]] = i
		for (int edge = 0; edge < edge::COUNT; edge++)
			into.eperm[c.eperm[edge]] = edge;
		for (int i = 0; i < corner::COUNT; i++)
			into.cori[i] = inv_cori[c.cori[into.cperm[i]]];
		for (int i = 0; i < edge::COUNT; i++)
			into.eori[i] = c.eori[into.eperm[i]];
	}

	int check(const cube& c) {
		bool corners[corner::COUNT] = {};
		int cori_sum = 0;

		for (int i = 0; i < corner::COUNT; i++) {
			if (c.cperm[i] < 0 || c.cperm[i] >= corner::COUNT)
				return 1; // invalid corner cubie
			corners[c.cperm[i]] = true;
			if (c.cori[i] < 0 || c.cori[i] >= 3)
				return 2; // invalid corner orientation
			cori_sum += c.cori[i];
		}
		if (cori_sum % 3 != 0)
			return 3; // invalid twist parity
		for (bool corner : corners) {
			if (!corner)
				return 4; // missing corner
		}

		bool edges[edge::COUNT] = {};
		int eori_sum = 0;

		for (int i = 0; i < edge::COUNT; i++) {
			if (c.eperm[i] < 0 || c.eperm[i] >= edge::COUNT)
				return 5; // invalid edge cubie
			edges[c.eperm[i]] = true;
			if (c.eori[i] < 0 || c.eori[i] >= 2)
				return 6; // invalid edge orientation
			eori_sum += c.eori[i];
		}
		if ((eori_sum & 1) != 0)
			return 7; // invalid flip parity
		for (bool edge : edges) {
			if (!edge)
				return 8; // missing edge
		}

		if (parity(c.cperm, corner::COUNT) != parity(c.eperm, edge::COUNT))
			return 9; // corner and edge permutation parity mismatch
		return 0;
	}

	void shuffle(cube& c) {
		for (int i = 0; i < corner::COUNT; i++)
			c.cperm[i] = i;
		for (int i = 0; i < edge::COUNT; i++)
			c.eperm[i] = i;

		coord::set_corners(c, std::uniform_int_distribution<int>(0, coord::N_CORNERS)(gen));
		std::shuffle(c.eperm, c.eperm + edge::COUNT, gen); // no coordinate for all edges
		if (parity(c.cperm, corner::COUNT) != parity(c.eperm, edge::COUNT))
			std::swap(c.cperm[corner::COUNT - 2], c.cperm[corner::COUNT - 1]); // flip parity

		coord::set_twist(c, std::uniform_int_distribution<int>(0, coord::N_TWIST - 1)(gen));
		coord::set_flip(c, std::uniform_int_distribution<int>(0, coord::N_FLIP - 1)(gen));
	}

	// We could maybe make this faster, but it is not performance critical anyways
	bool operator==(const cube& c1, const cube& c2) {
		return
			std::equal(c1.cperm, c1.cperm + corner::COUNT, c2.cperm) &&
			std::equal(c1.eperm, c1.eperm + edge::COUNT, c2.eperm) &&
			std::equal(c1.cori, c1.cori + corner::COUNT, c2.cori) &&
			std::equal(c1.eori, c1.eori + edge::COUNT, c2.eori)
			;
	}

	bool operator!=(const cube& c1, const cube& c2) {
		return !(c1 == c2);
	}

	int ctz64(uint64_t x) {
		int r = 63;
		x &= ~x + 1;
		if (x & 0x00000000FFFFFFFF) r -= 32;
		if (x & 0x0000FFFF0000FFFF) r -= 16;
		if (x & 0x00FF00FF00FF00FF) r -= 8;
		if (x & 0x0F0F0F0F0F0F0F0F) r -= 4;
		if (x & 0x3333333333333333) r -= 2;
		if (x & 0x5555555555555555) r -= 1;
		return r;
	}

	int ffsll(uint64_t x)
	{
		if (x == 0) return 0;
		return ctz64(x) + 1;
	}

	int ffs(int x)
	{
		int r = 1;

		if (!x)
			return 0;
		if (!(x & 0xffff)) {
			x >>= 16;
			r += 16;
		}
		if (!(x & 0xff)) {
			x >>= 8;
			r += 8;
		}
		if (!(x & 0xf)) {
			x >>= 4;
			r += 4;
		}
		if (!(x & 3)) {
			x >>= 2;
			r += 2;
		}
		if (!(x & 1)) {
			x >>= 1;
			r += 1;
		}
		return r;
	}
}
