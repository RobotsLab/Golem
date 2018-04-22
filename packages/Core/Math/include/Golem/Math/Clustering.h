/** @file Clustering.h
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#pragma once
#ifndef _GOLEM_MATH_CLUSTERING_H_
#define _GOLEM_MATH_CLUSTERING_H_

//------------------------------------------------------------------------------

#include <Golem/Defs/Constants.h>
#include <Golem/Math/Math.h>
#include <vector>
#include <map>
#include <algorithm>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Clustering algorithms */
template <typename _Size, typename _Value> class Clustering {
public:
	/** Size type */
	typedef _Size Size;
	/** Size 1D array */
	typedef std::vector<Size> SizeSeq;
	/** Size 2D array */
	typedef std::vector<SizeSeq> SizeSeqSeq;
	/** Value type */
	typedef _Value Value;
	/** Value 1D array */
	typedef std::vector<Value> ValueSeq;
	/** Value 2D array */
	typedef std::vector<ValueSeq> ValueSeqSeq;

	/** Allocate 2D array
	* @param size										size of array
	* @param array										array
	* @param value										initialisation value
	*/
	static void allocate(Size size, ValueSeqSeq& array, const Value& value = Value()) {
		array.resize(size);
		for (typename ValueSeqSeq::iterator i = array.begin(); i != array.end(); ++i)
			i->resize(size, value);
	}

	/** Initialise triangular array
	* @param size										size of array
	* @param array										array
	* @param value										initialisation value
	* @param func(i, j)									value of array[i][j] for i, j = [0..size), i != j, array[i][j] == array[j][i]
	*/
	template <typename _Func> static void initialise(Size size, ValueSeqSeq& array, const Value& value, _Func func) {
		// compute distances between data points
		allocate(size, array, value);
		for (Size i = 0; i < size - 1; ++i)
			for (Size j = i + 1; j < size; ++j)
				array[i][j] = array[j][i] = func(i, j);
	}

	/** 2D array dimensions
	* @param array										array
	* @return											array dimensions
	*/
	static Size dimensions(const ValueSeqSeq& array) {
		Size size = (Size)array.size();
		for (typename ValueSeqSeq::const_iterator i = array.begin(); i != array.end(); ++i)
			if (size > (Size)i->size())
				size = (Size)i->size();
		return size;
	}

	/** Median of array
	* @param array										array
	* @return											median value
	*/
	static Value median(ValueSeq& m) {
		const size_t n = m.size() / 2;
		std::nth_element(m.begin(), m.begin() + n, m.end());
		return m.size() % 2 ? m[n] : golem::numeric_const<Value>::HALF*(m[n - 1] + m[n]);
	}

	/** Median absolute deviation of triangular array
	* @param array										size x size array
	* @param median										median value
	* @param deviation									deviation value
	*/
	static void mad(const ValueSeqSeq& array, Value& median, Value& deviation) {
		const Size size = Clustering::dimensions(array);
		ValueSeq m(size*(size - 1) / 2);
		for (Size i = 0, k = 0; i < size - 1; ++i)
			for (Size j = i + 1; j < size; ++j, ++k)
				m[k] = array[i][j];
		median = Clustering::median(m);
		for (typename ValueSeq::iterator i = m.begin(); i != m.end(); ++i)
			*i = golem::Math::abs(median - *i);
		deviation = Clustering::median(m);
	}

	/** Partitioning Around Medoids
	* @param D											size x size distance array
	* @param clusters									number of clusters
	* @param steps										number of iterations
	* @param assignments								assignments[i] is the exemplar for data point i
	* @param cost										configuration cost
	* @param rand(size)									random integer generator [0..size)
	*/
	template <typename _Rand> static void pam(const ValueSeqSeq& D, Size clusters, Size steps, SizeSeq& assignments, Value& cost, _Rand rand) {
		const Size size = dimensions(D);

		SizeSeq medoids(size);
		SizeSeq assignmentsTest(size);
		Value costTest;

		SizeSeqSeq clusterAssignment(clusters);
		ValueSeq clusterCost(clusters);
		
		SizeSeq ca;

		cost = golem::numeric_const<Value>::MAX;
		for (Size step = 0; step < steps; ++step) {
			// generate <clusters> random numbers without repetitions using random permutation
			for (Size i = 0; i < size; ++i)
				medoids[i] = i;
			std::random_shuffle(medoids.begin(), medoids.end(), rand);

			// repeat until no improvements is possible
			for (bool hasImproved = true; hasImproved;) {
				// cluster parameters
				for (Size j = 0; j < clusters; ++j) {
					clusterAssignment[j].clear();
					clusterCost[j] = golem::numeric_const<Value>::ZERO;
				}

				// assign points to medoids, compute total cost
				costTest = golem::numeric_const<Value>::ZERO;
				for (Size i = 0; i < size; ++i) {
					Value dist = golem::numeric_const<Value>::MAX;
					Size k = 0;
					
					for (Size j = 0; j < clusters; ++j) {
						// allow to explain medoid itself: d = 0, i == medoids[j]
						const Value d = D[i][medoids[j]];
						if (dist > d) {
							dist = d;
							k = j;
						}
					}

					assignmentsTest[i] = medoids[k];
					costTest += dist;

					if (dist > golem::numeric_const<Value>::ZERO) {
						clusterAssignment[k].push_back(i);
						clusterCost[k] += dist;
					}
				}

				// test
				if (cost > costTest) {
					cost = costTest;
					assignments = assignmentsTest;
				}
				
				// improve clusters
				hasImproved = false;
				for (Size j = 0; j < clusters; ++j) {
					ca = clusterAssignment[j];
					Value cc = clusterCost[j];
					Size cm = size;
					
					// try to swap cluster medoid j with cluster member i
					const Size cmOld = medoids[j];
					for (Size i = 0; i < ca.size(); ++i) {
						const Size cmNew = ca[i];
						Value ccTest = D[cmNew][cmOld];
						for (Size k = 0; k < i; ++k)
							ccTest += D[cmNew][ca[k]];
						for (Size k = i + 1; k < ca.size(); ++k)
							ccTest += D[cmNew][ca[k]];
						if (cc > ccTest) {
							cc = ccTest;
							cm = i;
						}
					}

					// test for improvement
					if (cm < size) {
						medoids[j] = ca[cm];
						hasImproved = true;
					}
				}
			}
		}
	}

	/** Affinity Propagation
	* @param S											size x size similarity array
	* @param lambda										damping factor in [0, 1]
	* @param steps										maximum number of iterations
	* @param assignments								assignments[i] is the exemplar for data point i
	* @param convergence								convergence test
	*/
	template <typename _Convergence> static void affinity(const ValueSeqSeq& S, Value lambda, Size steps, SizeSeq& assignments, _Convergence convergence) {
		const Size size = dimensions(S);

		ValueSeqSeq R, A;
		allocate(size, R, golem::numeric_const<Value>::ZERO);
		allocate(size, A, golem::numeric_const<Value>::ZERO);

		SizeSeq exemplars;

		for (Size s = 0; s < steps; ++s) {
			// responsibility
#pragma omp parallel for
			for (Size i = 0; i < size; ++i) {
				for (Size j = 0; j < size; ++j) {
					Value r = - golem::numeric_const<Value>::MAX;

					for (Size k = 0; k < j; ++k) {
						const Value val = S[i][k] + A[i][k];
						if (r < val)
							r = val;
					}
					for (Size k = j + 1; k < size; ++k) {
						const Value val = S[i][k] + A[i][k];
						if (r < val)
							r = val;
					}
					r = S[i][j] - r;
					
					R[i][j] = (golem::numeric_const<Value>::ONE - lambda)*r + lambda*R[i][j];
				}
			}

			// availability
#pragma omp parallel for
			for (Size i = 0; i < size; ++i) {
				for (Size j = 0; j < size; ++j) {
					Value a = golem::numeric_const<Value>::ZERO;

					if (i == j) {
						for (Size k = 0; k < j; ++k)
							a += std::max(golem::numeric_const<Value>::ZERO, R[k][j]);
						for (Size k = j + 1; k < size; ++k)
							a += std::max(golem::numeric_const<Value>::ZERO, R[k][j]);
					}
					else {
						const Size k1 = std::min(i, j);
						const Size k2 = std::max(i, j);
						for (Size k = 0; k < k1; ++k)
							a += std::max(golem::numeric_const<Value>::ZERO, R[k][j]);
						for (Size k = k1 + 1; k < k2; ++k)
							a += std::max(golem::numeric_const<Value>::ZERO, R[k][j]);
						for (Size k = k2 + 1; k < size; ++k)
							a += std::max(golem::numeric_const<Value>::ZERO, R[k][j]);
						a = std::min(golem::numeric_const<Value>::ZERO, R[j][j] + a);
					}
					
					A[i][j] = (golem::numeric_const<Value>::ONE - lambda)*a + lambda*A[i][j];
				}
			}

			// find exemplars
			exemplars.clear();
			for (Size i = 0; i < size; ++i)
				if (R[i][i] + A[i][i] > golem::numeric_const<Value>::ZERO)
					exemplars.push_back(i);

			// check for convergence
			if (convergence(size, s, exemplars))
				break;
		}

		// data point assignment, assignments[i] is the exemplar for data point i
		assignments.resize(size, 0);
		for (Size i = 0; i < size; ++i) {
			Size assignment = 0;
			Value val = - golem::numeric_const<Value>::MAX;
			for (Size j = 0; j < Size(exemplars.size()); ++j) {
				const Size k = exemplars[j];
				if (S[i][k] > val) {
					val = S[i][k];
					assignment = k;
				}
			}
			assignments[i] = assignment;
		}
	}

	/** Optics density clustering
	* @param infinity									distance infinity
	* @param size										size of data set
	* @param density									minimum number of neighbours
	* @param radius										maximum size of neighbourhood
	* @param indices									data indices sorted by cluster
	* @param clusters									indices to indices pointing beginning of clusters
	* @param distFunc(index, neighbours, distances)		distance function computing neighbours sorted along their distances within radius from a given data index
	*/
	template <typename _DistFunc> static void optics(Value infinity, Size size, Size density, Value radius, SizeSeq& indices, SizeSeq& clusters, _DistFunc distFunc) {
		/** Node */
		class Node {
		public:
			/** Node sequence */
			typedef std::vector<Node> Seq;

			/** Reachability distance sorted container */
			class Seed {
			public:
				/** Valueance map */
				typedef std::multimap<Value, Size> ValueMap;
				/** Size map */
				typedef std::map<Size, typename ValueMap::iterator> SizeMap;

				/** Update seeds and nodes */
				void update(Size i, const SizeSeq& neighbours, const ValueSeq& distances, Seq& nodes) {
					const Node& inode = nodes[i];
					for (Size k = 0; k < neighbours.size(); ++k) {
						const Size j = neighbours[k];
						Node& jnode = nodes[j];
						if (!jnode.processed) {
							const Value distanceReachability = std::max(inode.distanceCore, distances[k]);
							if (contains(j))
								if (jnode.distanceReachability > distanceReachability)
									erase(j);
								else
									continue;
							insert(j, distanceReachability);
							jnode.distanceReachability = distanceReachability;
						}
					}
				}
				/** Retrieve index with the lowest reachability distance */
				Size next() {
					const Size index = distMap.begin()->second;
					erase(index);
					return index;
				}

				/** Insert */
				inline void insert(Size index, Value dist) {
					indexMap.insert(typename SizeMap::value_type(index, distMap.insert(typename ValueMap::value_type(dist, index))));
				}
				/** Erase */
				inline void erase(Size index) {
					distMap.erase(indexMap[index]);
					indexMap.erase(index);
				}
				/** Contains */
				inline bool contains(Size index) const {
					return indexMap.find(index) != indexMap.end();
				}
				/** No indices */
				inline bool empty() const {
					return indexMap.empty();
				}

			private:
				/** Valueance map */
				ValueMap distMap;
				/** Size map */
				SizeMap indexMap;
			};

			/** Processed */
			bool processed;
			/** Core distance */
			Value distanceCore;
			/** Reachability distance */
			Value distanceReachability;

			/** Default initialisation */
			Node(Value init) : processed(false), distanceCore(init), distanceReachability(init) {}
		};

		// compute distances
		indices.clear();
		typename Node::Seq nodes(size, Node(infinity));
		ValueSeq distances;
		SizeSeq neighbours;
		for (Size i = 0; i < size; ++i) {
			Node& inode = nodes[i];
			if (!inode.processed) {
				distFunc(i, neighbours, distances);
				inode.distanceCore = distances.size() < density ? infinity : distances[density - 1];
				inode.processed = true;
				indices.push_back(i);
				if (inode.distanceCore < infinity) {
					typename Node::Seed seeds;
					seeds.update(i, neighbours, distances, nodes);
					while (!seeds.empty()) {
						const Size j = seeds.next();
						Node& jnode = nodes[j];
						distFunc(j, neighbours, distances);
						jnode.distanceCore = distances.size() < density ? infinity : distances[density - 1];
						jnode.processed = true;
						indices.push_back(j);
						if (jnode.distanceCore < infinity)
							seeds.update(j, neighbours, distances, nodes);
					}
				}
			}
		}
		// extract clusters
		clusters.clear();
		Size j = 0;
		for (Size i = 0; i < indices.size(); ++i) {
			indices[j] = indices[i];
			const Node& jnode = nodes[indices[j]];
			if (jnode.distanceCore <= radius) {
				if (jnode.distanceReachability > radius)
					clusters.push_back(j);
				++j;
			}
		}
		indices.resize(j);
	}

	/** Local Outlier Factor
	* @param D											size x size distance array
	* @param neighbours									number of neighbours
	* @param lofs										local outlier factor for each data point
	*/
	static void lof(const ValueSeqSeq& D, Size neighbours, ValueSeq& lofs) {
		/** Neighbour */
		class Neighbour {
		public:
			/** 1D array */
			typedef std::vector<Neighbour> Seq;
			/** 2D array */
			typedef std::vector<Seq> SeqSeq;

			/** Index */
			Size index;
			/** Distance */
			Value distance;

			/** Neighbour */
			Neighbour(Size index = 0, Value distance = golem::numeric_const<Value>::ZERO) {
				set(index, distance);
			};

			/** Set values */
			inline void set(Size index, Value distance) {
				this->index = index;
				this->distance = distance;
			}

			/** Growing distance comparator */
			inline bool operator < (const Neighbour& n) const {
				return distance < n.distance;
			}
		};

		const Size size = dimensions(D);

		// compute neighbourhood
		typename Neighbour::SeqSeq N(size);
		typename Neighbour::Seq tmp(size);
		for (Size i = 0; i < size; ++i) {
			// distances i - j
			for (Size j = 0; j < size; ++j)
				tmp[j].set(j, D[i][j]);
			
			// sort, i - i distance is infinite
			std::nth_element(tmp.begin(), tmp.begin() + neighbours, tmp.end());
			N[i].assign(tmp.begin(), tmp.begin() + neighbours);
		}

		// local reachability density
		ValueSeq R(size);
		for (Size i = 0; i < size; ++i) {
			const typename Neighbour::Seq& n = N[i];

			Value d = golem::numeric_const<Value>::ZERO;
			for (Size j = 0; j < n.size(); ++j) {
				const Size k = n[j].index;
				d += std::max(N[k].back().distance, D[i][k]);
			}

			R[i] = n.size() / d;
		}

		// local outlier factor
		lofs.resize(size);
		for (Size i = 0; i < size; ++i) {
			const typename Neighbour::Seq& n = N[i];

			Value d1 = golem::numeric_const<Value>::ZERO, d2 = golem::numeric_const<Value>::ZERO;
			for (Size j = 0; j < n.size(); ++j) {
				const Size k = n[j].index;
				d1 += R[k];
				d2 += std::max(N[k].back().distance, D[i][k]);
			}

			lofs[i] = d1 * d2;
		}
	}

	/** F-test
	* @param size										size of data set
	* @param confidence									confidence value [0..1]
	* @param valueFunc(i)								value of data point i
	* @return											ftest density/index
	*/
	template <typename _ValueFunc> static Size ftest(Size size, Value confidence, _ValueFunc valueFunc) {
		ValueSeq value(size);
		Value norm = golem::numeric_const<Value>::MAX;
		for (Size i = 0; i < size; ++i) {
			value[i] = valueFunc(i);
			if (norm > value[i])
				norm = value[i];
		}

		ValueSeq cdf(size);
		cdf[0] = value[0] - norm;
		for (Size i = 1; i < size; ++i)
			cdf[i] = cdf[i - 1] + value[i] - norm;
		
		Size index = 0;
		for (; index + 1 < size && cdf[index] < confidence*cdf.back(); ++index);

		return index;
	}
};

//------------------------------------------------------------------------------

};	// namespace golem

#endif /*_GOLEM_MATH_CLUSTERING_H_*/
