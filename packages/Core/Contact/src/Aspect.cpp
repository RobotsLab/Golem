/** @file Aspect.cpp
 *
 * Contact view aspect
 *
 * @author Marek Kopicki
 *
 * @copyright  Copyright (C) 2016 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

//------------------------------------------------------------------------------

#include <Golem/Contact/Aspect.h>
#include <Golem/Tools/Cloud.h>
#include <Golem/Sys/XMLData.h>
#include <Golem/Plugin/Data.h>
#include <Golem/Tools/Menu.h>
#include <unordered_map>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

void golem::Aspect::Data::Appearance::load(const golem::XMLContext* xmlcontext) {
	golem::XMLData(referenceFramePair, xmlcontext->getContextFirst("reference_frame_pair"), false);
	golem::XMLData(referenceFrameView, xmlcontext->getContextFirst("reference_frame_view"), false);
	golem::XMLData(referenceFrame, xmlcontext->getContextFirst("reference_frame"), false);
	golem::XMLData("frame_show", frameShow, const_cast<golem::XMLContext*>(xmlcontext), false);
	golem::XMLData(frameSize, xmlcontext->getContextFirst("frame_size"), false);
	golem::XMLData(frameTrn, xmlcontext->getContextFirst("frame_trn"), false);
	golem::XMLData(frameTrnComplete, xmlcontext->getContextFirst("frame_trn_complete"), false);
	golem::XMLData(frameTrnPartial, xmlcontext->getContextFirst("frame_trn_partial"), false);
	golem::XMLData("normal_len", normalLen, const_cast<golem::XMLContext*>(xmlcontext), false);
	golem::XMLData(pointColour, xmlcontext->getContextFirst("point_colour"), false);
	golem::XMLData(pointSimColour, xmlcontext->getContextFirst("point_sim_colour"), false);
	golem::XMLData("point_size", pointSize, const_cast<golem::XMLContext*>(xmlcontext), false);
}

bool golem::Aspect::Data::empty() const {
	return contactPartial.empty() || contactComplete.empty() || partialMap.empty() || completeMap.empty() || simMat.empty();
}

void golem::Aspect::Data::clear() {
	contactPartial.clear();
	contactComplete.clear();
	partialMap.clear();
	completeMap.clear();
	simMat.clear();
	viewDataMap.clear();
	pointsMap.clear();
}

void golem::Aspect::Data::draw(const Appearance& appearance, const data::ContactModel::Data& processedData, golem::DebugRenderer& renderer) const {
	renderer.setPointSize(appearance.pointSize);
	
	// over compressed contacts
	Mat34 trn = appearance.referenceFrame;
	for (Contact3D::Data::Map::const_iterator icontact = processedData.contacts.begin(); icontact != processedData.contacts.end(); ++icontact) {
		renderer.addAxes3D(trn, appearance.frameSize);
		Contact3D::drawPoints(icontact->second.model, appearance.pointColour, appearance.normalLen, trn, renderer);

		// uncompressed/complete contacts
		Mat34 trnComplete = trn;
		for (CompleteMap::const_iterator j = completeMap.find(icontact->first), jend = completeMap.upper_bound(icontact->first); j != jend; ++j) {
			Contact3D::Data::Map::const_iterator jcontact = contactComplete.find(j->second.first);
			if (jcontact == contactComplete.end())
				continue;
			trnComplete.p += appearance.frameTrnComplete;

			Mat34 localFrame = trnComplete * j->second.second;
			//Mat34 localFrame;
			//localFrame.setInverse(j->second.second);
			//localFrame = trnComplete * localFrame;
			renderer.addAxes3D(localFrame, appearance.frameSize);
			Contact3D::drawPoints(icontact->second.model, appearance.pointColour, appearance.normalLen, localFrame, renderer);

			renderer.addAxes3D(trnComplete, appearance.frameSize);
			Contact3D::drawPoints(jcontact->second.model, appearance.pointSimColour, appearance.normalLen, trnComplete, renderer);

			// uncompressed/partial contacts
			Mat34 trnPartial = trnComplete;
			for (PartialMap::const_iterator k = partialMap.find(jcontact->first), kend = partialMap.upper_bound(jcontact->first); k != kend; ++k) {
				Contact3D::Data::Map::const_iterator kcontact = contactPartial.find(k->second);
				if (kcontact == contactPartial.end())
					continue;

				trnPartial.p += appearance.frameTrnPartial;
				renderer.addAxes3D(trnPartial, appearance.frameSize);
				Contact3D::drawPoints(kcontact->second.model, appearance.pointColour, appearance.normalLen, trnPartial, renderer);
			}
		}

		trn.p += appearance.frameTrn;
	}
}

//------------------------------------------------------------------------------

void golem::Aspect::Desc::load(const golem::XMLContext* xmlcontext) {
	golem::XMLData("enabled", enabled, const_cast<golem::XMLContext*>(xmlcontext), false);
	golem::XMLData("debug_level", debugLevel, const_cast<golem::XMLContext*>(xmlcontext), false);

	dataAppearance.load(xmlcontext->getContextFirst("appearance"));

	golem::XMLData(contactDist, xmlcontext->getContextFirst("contact distance"), false);

	golem::XMLData("enabled", contactSimOpt, xmlcontext->getContextFirst("contact clustering optimisation"), false);
	golem::XMLData("lin_scale", contactSimLinScale, xmlcontext->getContextFirst("contact clustering"), false);
	contactSimOptDesc.load(xmlcontext->getContextFirst("contact clustering optimisation"));
	golem::XMLData("init_trials", contactSimOptInitTrials, xmlcontext->getContextFirst("contact clustering optimisation"), false);

	golem::XMLData("neighbours_fac", contactOutlierNeighboursFac, xmlcontext->getContextFirst("contact clustering outliers"), false);
	golem::XMLData("ftest", contactOutlierFTest, xmlcontext->getContextFirst("contact clustering outliers"), false);

	contactClusteringDesc.load(xmlcontext->getContextFirst("contact clustering"));

	golem::XMLData("subsample", contactExemplarSubsample, xmlcontext->getContextFirst("contact clustering exemplar"), false);
	golem::XMLData("dist_fac", contactExemplarSubsampleDistFac, xmlcontext->getContextFirst("contact clustering exemplar"), false);

	golem::XMLData("enabled", viewPruning, xmlcontext->getContextFirst("view pruning"), false);
	golem::XMLData("min_contacts", viewMinContacts, xmlcontext->getContextFirst("view pruning"), false);
	viewSolverDesc->load(xmlcontext->getContextFirst("view pruning solver"));

	golem::XMLData("grid_leaf_size", viewDownsampleGridLeafSize, xmlcontext->getContextFirst("view downsample"), false);

	viewClusteringDesc.load(xmlcontext->getContextFirst("view clustering"));
}

//------------------------------------------------------------------------------

Aspect::Aspect(const Desc& desc, const Configuration& configuration) : configuration(configuration), manipulator(configuration.getManipulator()), context(const_cast<golem::Context&>(manipulator.getContext())), parallels(context.getParallels()), rand(context.getRandSeed()) {
	desc.assertValid(Assert::Context("Aspect()."));

	// check parallels
	if (!parallels || parallels->getNumOfThreads() < 1)
		throw golem::Message(golem::Message::LEVEL_CRIT, "Aspect(): Parallels required");

	this->desc = desc;

	this->viewSolver = desc.viewSolverDesc->create(const_cast<Manipulator&>(manipulator));
}

//------------------------------------------------------------------------------

void Aspect::process(golem::data::ContactModel::Data& data, golem::Aspect::Data& aspectData, UICallback* pUICallback) {
	// data preparation
	preprocessData(data, aspectData);

	// contact processing
	processContacts(data, aspectData, pUICallback);

	// view processing
	processViews(data, aspectData, pUICallback);

	// compact data
	compactData(data, aspectData);

	// debug
	if (desc.debugLevel >= 1)
		print("Aspect::process()", data, aspectData);
}

//------------------------------------------------------------------------------

void golem::Aspect::compactData(golem::data::ContactModel::Data& data, golem::Aspect::Data& aspectData) {
	// compact contact indices
	U32 index = 0;
	Contact3D::Data::Map contacts;
	Aspect::Data::CompleteMap completeMap;
	for (Contact3D::Data::Map::const_iterator i = data.contacts.begin(); i != data.contacts.end(); ++i) {
		// update contacts pointers
		bool isUsed = false;
		for (Contact::View::Seq::iterator j = data.views.begin(); j != data.views.end(); ++j)
			for (Contact::View::ModelPtr::Map::iterator k = j->models.begin(); k != j->models.end(); ++k)
				if (k->second.index == i->first) {
					k->second.index = index;
					isUsed = true;
				}
		if (!isUsed)
			continue;
		// update contacts
		contacts.insert(std::make_pair(index, i->second));
		// update aspect data
		const std::pair<Aspect::Data::CompleteMap::const_iterator, Aspect::Data::CompleteMap::const_iterator> range = aspectData.completeMap.equal_range(i->first);
		for (Aspect::Data::CompleteMap::const_iterator j = range.first; j != range.second; ++j)
			completeMap.insert(std::make_pair(index, j->second));
		// debug
		if (desc.debugLevel >= 1)
			context.debug("Aspect::compactData(): re-mapping contact %u -> %u\n", i->first + 1, index + 1);
		// increase index
		++index;
	}
	data.contacts = contacts;
	aspectData.completeMap = completeMap;
}

void golem::Aspect::preprocessData(golem::data::ContactModel::Data& data, golem::Aspect::Data& aspectData) {
	// clear
	aspectData.contactPartial.clear();
	aspectData.contactComplete.clear();
	aspectData.partialMap.clear();
	aspectData.completeMap.clear();

	// sort views, complete views first
	Contact::View::PtrSeq ptrviews;
	for (Contact::View::Seq::iterator i = data.views.begin(); i != data.views.end(); ++i)
		ptrviews.push_back(&*i);
	std::sort(ptrviews.begin(), ptrviews.end(), Contact::View::compare_first());

	// Register views only once
	const bool registerViews = aspectData.pointsMap.empty();

	// replace partial view with complete
	Contact::View::Seq views;
	for (Contact::View::PtrSeq::iterator i = ptrviews.begin(), j = ptrviews.end(); i != ptrviews.end(); ++i) {
		if ((*i)->hasCompleteView()) {
			// memorise the complete view
			j = i;

			// for all contact models (complete view)
			for (Contact::View::ModelPtr::Map::const_iterator k = (*i)->models.begin(); k != (*i)->models.end(); ++k) {
				// copy model
				Contact3D::Data::Map::const_iterator model = data.contacts.find(k->second.index);
				if (model == data.contacts.end())
					throw Message(Message::LEVEL_ERROR, "Aspect::preprocessData(): unable to find complete contact model #%u", k->second.index + 1);
				aspectData.contactComplete.insert(std::make_pair(k->second.index, model->second));
				// no compression at this stage, 1 to 1 map
				aspectData.completeMap.insert(std::make_pair(k->second.index, std::make_pair(k->second.index, Mat34::identity())));
			}

			// add complete view in case there is no partial views for this object
			views.push_back(**i);
		}
		else {
			if (j == ptrviews.end() || (*j)->getObjectIndex() != (*i)->getObjectIndex())
				throw Message(Message::LEVEL_ERROR, "Aspect::preprocessData(): unable to find complete view contacts");

			// for all contact models (partial view)
			for (Contact::View::ModelPtr::Map::iterator k = (*i)->models.begin(); k != (*i)->models.end(); ++k) {
				// find corresponding contact model of a complete view
				// assume there is one model per link at this stage
				Contact::View::ModelPtr::Map::const_iterator l = (*j)->models.find(k->first);
				if (l == (*j)->models.end())
					throw Message(Message::LEVEL_ERROR, "Aspect::preprocessData(): unable to find complete contact for link #%u of object #%u", k->first.toString().c_str(), (*i)->getObjectIndex());

				// copy model
				Contact3D::Data::Map::const_iterator model = data.contacts.find(k->second.index);
				if (model == data.contacts.end())
					throw Message(Message::LEVEL_ERROR, "Aspect::preprocessData(): unable to find partial contact model #%u of object #%u", k->second.index + 1, (*i)->getObjectIndex());
				aspectData.contactPartial.insert(std::make_pair(k->second.index, model->second));
				// no compression at this stage, 1 to many map
				aspectData.partialMap.insert(std::make_pair(l->second.index, k->second.index));

				// remove partial view models
				data.contacts.erase(k->second.index);

				// replace contact model - re-index
				k->second.index = l->second.index;
				//k->second.weight = l->second.weight;
				k->second.frame.setId(); // id, no compression yet
			}

			// remove complete view if it was not removed yet
			if (!views.empty() && views.back().hasCompleteView())
				views.pop_back();

			// add partial view
			views.push_back(**i);
		}

		// debug info
		if (desc.debugLevel >= 1)
			context.debug("Aspect::preprocessData(): %s_view=#%u/%u, object=#%u, points=%s, models=%u\n", (*i)->hasCompleteView() ? "complete" : "partial", U32(i - ptrviews.begin()) + 1, U32(ptrviews.size()), (*i)->getObjectIndex(), (*i)->point3D ? std::to_string((*i)->point3D->getSize()).c_str() : "unavailable", U32((*i)->models.size()));
	}

	// replace views
	data.views = views;

	// downsample views
	if (registerViews) {
		for (Contact::View::PtrSeq::iterator i = ptrviews.begin(), j = ptrviews.end(); i != ptrviews.end(); ++i) {
			// view registration for view pruning procedure
			const data::Point3D* point3D = (*i)->point3D;
			if (point3D) {
				F32Vec3Seq& points = aspectData.pointsMap[(*i)->getObjectIndex()]; // create if empty
				for (size_t i = 0; i < point3D->getSize(); ++i)
					points.push_back(F32Vec3((const golem::data::Point3D::Vec3&)point3D->getPoint(i)));

			}
			else {
				context.warning("Aspect::preprocessData(): no Point3D interface provided for object #%u\n", (*i)->getObjectIndex());
				aspectData.pointsMap.erase((*i)->getObjectIndex());
			}
		}

		Cloud::DownsampleDesc downsampleDesc;
		downsampleDesc.enabledWithNormals = true;
		downsampleDesc.gridLeafSize = (float)desc.viewDownsampleGridLeafSize;
		Cloud::PointSeq inp, out;

		for (Aspect::Data::PointsMap::iterator i = aspectData.pointsMap.begin(); i != aspectData.pointsMap.end(); ++i) {
			// convert input point cloud
			inp.clear();
			inp.reserve(i->second.size());
			for (F32Vec3Seq::const_iterator j = i->second.begin(); j != i->second.end(); ++j) {
				Cloud::Point point;
				Cloud::convertPointXYZ(*j, point);
				Cloud::convertNormal(Vec3(REAL_ZERO, REAL_ZERO, REAL_ONE), point);
				Cloud::convertRGBA(RGBA::BLACK, point);
				inp.push_back(point);
			}
			
			// downsample
			context.debug("Aspect::preprocessData(): procesing views of object #%u/%u\n", i->first, (U32)aspectData.pointsMap.size());
			Cloud::downsampleWithNormals(context, downsampleDesc, inp, out);

			// convert output point cloud
			i->second.clear();
			for (Cloud::PointSeq::const_iterator j = out.begin(); j != out.end(); ++j) {
				F32Vec3 point;
				Cloud::convertPointXYZ(*j, point);
				i->second.push_back(point);
			}
		}
	}
}

//------------------------------------------------------------------------------

void golem::Aspect::processContacts(golem::data::ContactModel::Data& data, golem::Aspect::Data& aspectData, UICallback* pUICallback) {
	// Size
	U32 Size = (U32)data.contacts.size();
	if (Size < 2)
		throw Message(Message::LEVEL_ERROR, "Aspect::processContacts(): at least two contacts are required");

	// create random access ordered collection of contacts
	Contact3D::Data::SeqMap contactSeqMap;
	for (Contact3D::Data::Map::iterator i = data.contacts.begin(); i != data.contacts.end(); ++i)
		contactSeqMap.push_back(i);

	// if there is a similarity matrix available
	if (aspectData.simMat.empty() || pUICallback && pUICallback->hasInputEnabled() && Menu(context, *pUICallback).option(0, "Compute similarity matrix: ", { "YES", "NO" }) == 0) {
		// optimisation

		//std::unordered_map<golem::U32, data::Feature3D::Feature> stdDevMap;
		//std::for_each(data.contacts.begin(), data.contacts.end(), [&] (const Contact3D::Data::Map::value_type& val) { stdDevMap[val.first] = val.second.feature3DProperty.covarianceSqrt; });
		//ScopeGuard parameterGuard([&] () {
		//	std::for_each(stdDevMap.begin(), stdDevMap.end(), [&] (const std::pair<golem::U32, data::Feature3D::Feature>& val) { data.contacts[val.first].feature3DProperty.setStdDev(val.second); });
		//});

		if (desc.debugLevel >= 1 && pUICallback && pUICallback->hasInputEnabled()) {
			Menu menu(context, *pUICallback);
			
			desc.contactSimOpt = menu.option(desc.contactSimOpt ? 0 : 1, "Align contact frames: ", { "YES", "NO" }) == 0;
			
			Real dist;
			dist = Math::sqrt(desc.contactDist.featureDist);
			menu.readNumber("Enter feature distance: ", dist);
			desc.contactDist.featureDist = Math::sqr(dist);

			dist = Math::sqrt(desc.contactDist.featureDistMax);
			menu.readNumber("Enter feature distance maximum: ", dist);
			desc.contactDist.featureDistMax = Math::sqr(dist);
		}

		//std::for_each(data.contacts.begin(), data.contacts.end(), [&] (Contact3D::Data::Map::value_type& val) {
		//	data::Feature3D::Feature stdDev(val.second.feature3DProperty.covarianceSqrt.size());
		//	stdDev.multiply(desc.contactSimFeatureStdDevDist, val.second.feature3DProperty.covarianceSqrt);
		//	val.second.feature3DProperty.setStdDev(stdDev);
		//});

		// prepare auxiliary data
		SimPoint::SeqSeq pointSeqSeq(Size);
		SimVec3Seq meanSeq(Size);
		SimRealSeq diamSeq(Size);
		for (size_t i = 0; i < Size; ++i) {
			const Contact3D::Seq& contacts = contactSeqMap[i]->second.model;
			SimPoint::Seq& pointSeq = pointSeqSeq[i];
			pointSeq.resize(contacts.size());

			SimVec3 mean(golem::numeric_const<SimReal>::ZERO);
			for (size_t j = 0; j < pointSeq.size(); ++j) {
				pointSeq[j].set(contacts[j]);
				// update mean
				mean += pointSeq[j].point;
			}
			mean *= golem::numeric_const<SimReal>::ONE / pointSeq.size();
			meanSeq[i] = mean;

			SimReal diam = golem::numeric_const<SimReal>::ZERO;
			for (size_t j = 0; j < pointSeq.size(); ++j)
				diam = std::max(diam, mean.distanceSqr(pointSeq[j].point));
			diamSeq[i] = Math::sqrt(diam);
		}

		// contact cluster similarity and local frame
		ClusteringSim::allocate(Size, aspectData.simMat, Similarity(golem::numeric_const<golem::Real>::ZERO, golem::Mat34::identity()));

		// find mutual similarities
		CriticalSection cs;
		U32 i = 0, j = 0, k = 0; // i - column, j - row
		ParallelsTask(parallels, [&] (ParallelsTask*) {
			const U32 jobId = parallels->getCurrentJob()->getJobId();
			Rand rand(RandSeed(this->context.getRandSeed()._U32[0] + jobId, (U32)0));
			std::string msg;

			for (U32 ii, jj, kk;;) {
				{
					CriticalSectionWrapper csw(cs);
					if (j + 1 < Size)
						j = j + 1;
					else if (i + 2 < Size) {
						i = i + 1;
						j = i + 1;
					}
					else
						break;

					// i = 0..Size, j = 1..Size, j > i for any i,j
					jj = j;
					ii = i;
				}

				// compute i - j similarity and local SO(3) transformation
				contactSim(rand, ii, jj, contactSeqMap, pointSeqSeq, meanSeq, diamSeq, aspectData.simMat[ii][jj], msg);

				// debug
				if (desc.debugLevel >= 1) {
					{
						CriticalSectionWrapper csw(cs);
						kk = ++k;
					}
					context.debug("Aspect::processContacts(): step=%u/%u, contact pair=(%u, %u), similarity=%e, %s\n", kk, Size*(Size - 1) / 2, contactSeqMap[ii]->first + 1, contactSeqMap[jj]->first + 1, aspectData.simMat[ii][jj].first, msg.c_str());
				}

				// similarity relation is symmetrical for values
				aspectData.simMat[jj][ii].first = aspectData.simMat[ii][jj].first;
				// inverse for frames
				aspectData.simMat[jj][ii].second.setInverse(aspectData.simMat[ii][jj].second);
			}
		});
	}

	if (!aspectData.procCompleteMap.empty() && !aspectData.procContacts.empty() && !aspectData.procViews.empty() && pUICallback && pUICallback->hasInputEnabled() && Menu(context, *pUICallback).option(0, "Aspect::processContacts(): compute cluster contacts: ", { "YES", "NO" }) == 1) {
		aspectData.completeMap  = aspectData.procCompleteMap;
		data.contacts = aspectData.procContacts;
		data.views = aspectData.procViews;
		return;
	}

	// make a local copy, preserve complete matrix
	ClusteringSim::ValueSeqSeq simMat = aspectData.simMat;

	if (Size != (U32)simMat.size())
		throw Message(Message::LEVEL_ERROR, "Aspect::processContacts(): invalid similarity matrix size %u != %u", (U32)simMat.size(), Size); // could happen if data has been imported

	// outlier removal
	golem::U32 contactOutlierNeighbours = std::max((U32)1, (U32)(desc.contactOutlierNeighboursFac*Size));
	golem::Real contactOutlierFTest = desc.contactOutlierFTest;
	if (contactOutlierNeighbours > 0 && desc.debugLevel >= 2 && pUICallback && pUICallback->hasInputEnabled()) {
		Menu menu(context, *pUICallback);
		menu.readNumber("Enter outlier neighbours: ", contactOutlierNeighbours);
		menu.readNumber("Enter outlier f-test confidence: ", contactOutlierFTest);
	}

	if (contactOutlierNeighbours > 0 && contactOutlierFTest > REAL_EPS) {
		golem::U32 contactOutliers = 0;
		typedef std::vector< std::pair<U32, golem::Real> > FactorSeq;
		FactorSeq factors(Size);

		// distance array
		Clustering::ValueSeqSeq distance;
		Clustering::initialise(Size, distance, REAL_ZERO, [=] (U32 i, U32 j) -> golem::Real { return -simMat[i][j].first; });

		// clustering
		Clustering::ValueSeq lofs;
		Clustering::lof(distance, contactOutlierNeighbours, lofs);
		for (U32 i = 0; i < Size; ++i)
			factors[i] = std::make_pair(i, lofs[i]);
		std::sort(factors.begin(), factors.end(), [] (const FactorSeq::value_type& l, const FactorSeq::value_type& r) -> bool { return l.second > r.second; });

		// f-test
		contactOutliers = Size - 1 - Clustering::ftest(Size, contactOutlierFTest, [=] (U32 i) -> Real { return factors[Size - 1 - i].first; });

		// debug
		if (desc.debugLevel >= 1) {
			for (U32 i = 0; i < (U32)factors.size(); ++i)
				context.debug("Aspect::processContacts(): %s=%u/%u, index=%u/%u, LOF=%f\n", i < contactOutliers ? "outlier" : "inlier", (i < contactOutliers ? i : i - contactOutliers) + 1, (i < contactOutliers ? contactOutliers : Size - contactOutliers), factors[i].first, contactSeqMap[factors[i].first]->first + 1, factors[i].second);
		}

		// remove contact outliers
		if (contactOutliers > 0) {
			Clustering::SizeSeq indices(contactOutliers);
			for (U32 i = 0; i < contactOutliers; ++i)
				indices[i] = factors[i].first;
			std::sort(indices.begin(), indices.end(), [] (const Clustering::SizeSeq::value_type& l, const Clustering::SizeSeq::value_type& r) -> bool { return l > r; });

			// erase data
			for (U32 i = 0; i < Size; ++i)
				for (size_t j = 0; j < indices.size(); ++j)
					simMat[i].erase(simMat[i].begin() + indices[j]);
			for (size_t j = 0; j < indices.size(); ++j) {
				simMat.erase(simMat.begin() + indices[j]);

				Contact3D::Data::Map::iterator ptr = contactSeqMap[indices[j]];
				for (Contact::View::Seq::iterator l = data.views.begin(); l != data.views.end(); ++l)
					for (Contact::View::ModelPtr::Map::iterator m = l->models.begin(); m != l->models.end();) {
						Contact::View::ModelPtr::Map::iterator n = m++;
						if (n->second.index == ptr->first)
							l->models.erase(n);
					}

				data.contacts.erase(ptr);
				contactSeqMap.erase(contactSeqMap.begin() + indices[j]);
			}
			Size -= contactOutliers;
		}
	}

	// contact clustering
	Clustering::SizeSeq assignments;
	clustering(desc.contactClusteringDesc, Size, [=] (U32 i, U32 j) -> golem::Real { return simMat[i][j].first; }, assignments, pUICallback);

	// construct exemplar -> cluster map
	typedef std::map<U32, U32Seq> AssignmentMap;
	AssignmentMap assignmentMap;
	for (U32 i = 0; i < (U32)assignments.size(); ++i) {
		assignmentMap[assignments[i]];
		if (assignments[i] != i)
			assignmentMap[assignments[i]].push_back(i);
		//context.write("%u/%u -> %u/%u\n", contactSeqMap[i]->first, i, contactSeqMap[assignments[i]]->first, assignments[i]);
	}

	// make sure assignments are not in the other clusters
	for (AssignmentMap::const_iterator i = assignmentMap.begin(); i != assignmentMap.end(); ++i)
		for (AssignmentMap::iterator j = assignmentMap.begin(); j != assignmentMap.end(); ++j)
			for (size_t k = 0; k < j->second.size();)
				if (j->second[k] == i->first) {
					j->second.erase(j->second.begin() + k);
					context.debug("Aspect::processContacts(): cluster exemplar %u/%u removed from %u/%u\n", contactSeqMap[i->first]->first + 1, i->first + 1, contactSeqMap[j->first]->first + 1, j->first + 1);
				}
				else
					++k;

	// debug
	if (desc.debugLevel >= 1) {
		for (auto &i : assignmentMap) {
			std::string str;
			for (auto &j : i.second) str += std::to_string(contactSeqMap[j]->first + 1) + "/" + std::to_string(j + 1) + ", ";
			context.debug("Aspect::processContacts(): assignment: %u/%u -> {%s}\n", contactSeqMap[i.first]->first + 1, i.first + 1, str.c_str());
		}
	}

	// compress contacts
	for (AssignmentMap::const_iterator i = assignmentMap.begin(); i != assignmentMap.end(); ++i) {
		// compute new contact cluster
		golem::Contact3D::Seq contactSeq;
		contactCluster(rand, i->first, i->second, contactSeqMap, simMat, contactSeq);
		// set new contact cluster
		contactSeqMap[i->first]->second.model = contactSeq;
		const U32 exemplarIndex = contactSeqMap[i->first]->first;
		// replace other contacts
		Aspect::Data::CompleteMap completeMap;
		for (U32Seq::const_iterator j = i->second.begin(); j != i->second.end(); ++j) {
			const golem::Contact3D::Data::Map::iterator k = contactSeqMap[*j];
			const U32 contactIndex = k->first;
			const Mat34 contactFrame = simMat[i->first][*j].second;
			// remove contact
			data.contacts.erase(k);
			// replace contact indicies: views
			for (Contact::View::Seq::iterator l = data.views.begin(); l != data.views.end(); ++l)
				for (Contact::View::ModelPtr::Map::iterator m = l->models.begin(); m != l->models.end(); ++m)
					if (m->second.index == contactIndex) {
						m->second.index = exemplarIndex; // index
						m->second.frame = contactFrame; // local frame
					}
			// replace contact indicies: aspect data, complete map
			completeMap.insert(std::make_pair(exemplarIndex, std::make_pair(contactIndex, contactFrame)));
			aspectData.completeMap.erase(contactIndex); // by key
		}
		aspectData.completeMap.insert(completeMap.begin(), completeMap.end());
	}

	// update
	aspectData.procCompleteMap = aspectData.completeMap;
	aspectData.procContacts = data.contacts;
	aspectData.procViews = data.views;
}

void golem::Aspect::processViews(golem::data::ContactModel::Data& data, golem::Aspect::Data& aspectData, UICallback* pUICallback) {
	// prunning based on collisions
	if (desc.viewPruning)
		viewsPruning(data, aspectData, pUICallback);

	// prunning based on minimum number of contacts
	typedef std::vector<const Contact::View*> ViewRank;
	ViewRank viewRank;
	for (Contact::View::Seq::iterator i = data.views.begin(); i != data.views.end(); ++i)
		viewRank.push_back(&*i);
	// rank views with respect to number of models
	std::sort(viewRank.begin(), viewRank.end(), [] (const ViewRank::value_type& l, const ViewRank::value_type& r) -> bool { return l->models.size() > r->models.size(); });
	// prune views with too small number of contacts
	if (viewRank.empty())
		throw Message(Message::LEVEL_ERROR, "Aspect::processViews(): no available views");
	// at least one view must be left
	for (ViewRank::iterator i = viewRank.begin() + 1; i != viewRank.end(); ++i)
		if ((*i)->models.size() < desc.viewMinContacts) {
			context.debug("Aspect::processViews(): removing view #%u: insufficient number of contacts %u < %u for object #%u\n", (U32)(*i - data.views.data()) + 1, (U32)(*i)->models.size(), desc.viewMinContacts, (*i)->getObjectIndex());
			*i = nullptr;
		}
	// rank views with respect to the original index
	std::sort(viewRank.begin(), viewRank.end(), [] (const ViewRank::value_type& l, const ViewRank::value_type& r) -> bool { return l < r; });
	Contact::View::Seq views;
	for (ViewRank::iterator i = viewRank.begin(); i != viewRank.end(); ++i)
		if (*i) {
			context.debug("Aspect::processViews(): adding view #%u: number of contacts %u >= %u for object #%u\n", (U32)(*i - data.views.data()) + 1, (U32)(*i)->models.size(), desc.viewMinContacts, (*i)->getObjectIndex());
			views.push_back(**i);
		}

	// replace views
	data.views = views;
}

void golem::Aspect::print(const std::string& str, const golem::data::ContactModel::Data& data, const golem::Aspect::Data& aspectData) const {
	std::string tmp;

	// contacts
	for (auto &i : data.contacts) tmp += std::to_string(i.first + 1) + ", ";
	context.write("%s: Contacts (%u): {%s}\n", str.c_str(), (U32)data.contacts.size(), tmp.c_str());
	for (auto &i : data.contacts)
		if (i.second.type == Contact3D::TYPE_FEATURE)
			context.write("%s: Contact %u: feature_{dim=%u, size=%u, mean=(%s), std_dev=(%s)}\n",
				str.c_str(), i.first + 1, (U32)i.second.feature3DProperty.mean.size(), (U32)i.second.model.size(), data::Feature3D::to_string(i.second.feature3DProperty.mean, "%.3e ").c_str(), data::Feature3D::to_string(i.second.feature3DProperty.covarianceSqrt, "%.3e ").c_str());
	tmp.clear();
	for (auto &i : aspectData.completeMap) tmp += "(" + std::to_string(i.first + 1) + ", " + std::to_string(i.second.first + 1) + ") ";
	context.write("%s: Contacts map (%u): {%s}\n", str.c_str(), (U32)aspectData.completeMap.size(), tmp.c_str());
	
	// views
	for (auto &i : data.views) {
		tmp.clear();
		for (auto &j : i.models) tmp += "(" + j.first.toString() + ", " + std::to_string(j.second.index + 1) + ") ";
		context.write("%s: Views (%u): {%s}\n", str.c_str(), (U32)data.views.size(), tmp.c_str());
	}

	// point clouds
	tmp.clear();
	for (auto &i : aspectData.pointsMap) tmp += "(" + std::to_string(i.first) + ", " + std::to_string(i.second.size()) + ") ";
	context.write("%s: Registered point clouds (%u): {%s}\n", str.c_str(), (U32)aspectData.pointsMap.size(), tmp.c_str());

	// partial contacts
	tmp.clear();
	for (auto &i : aspectData.contactPartial) tmp += std::to_string(i.first + 1) + ", ";
	context.write("%s: Partial contacts (%u): {%s}\n", str.c_str(), (U32)aspectData.contactPartial.size(), tmp.c_str());
	tmp.clear();
	for (auto &i : aspectData.partialMap) tmp += "(" + std::to_string(i.first + 1) + ", " + std::to_string(i.second + 1) + ") ";
	context.write("%s: Partial contacts map (%u): {%s}\n", str.c_str(), (U32)aspectData.partialMap.size(), tmp.c_str());
}

//------------------------------------------------------------------------------

void golem::Aspect::clustering(ClusteringDesc& clusteringDesc, const golem::U32 Size, SimFunc simFunc, Clustering::SizeSeq& assignments, UICallback* pUICallback) const {
	// test: no compression - 1 to 1
	//for (U32 i = 0; i < (U32)contactSeqMap.size(); ++i) assignments[i] = i; return;

	bool none = false;

	// clustering algorithm
	if (desc.debugLevel >= 1 && pUICallback && pUICallback->hasInputEnabled()) {
		bool * option [] = { &clusteringDesc.affinityEnabled , &clusteringDesc.pamEnabled, &none };
		StringSeq optionStr = { "Affinity", "PAM", "None" };
		size_t index = 0;
		for (; index + 1 < sizeof(option) / sizeof(bool*) && !*option[index]; ++index);
		index = Menu(context, *pUICallback).option(index, "Clustering algorithm: ", optionStr);
		for (size_t i = 0; i < sizeof(option) / sizeof(bool*); ++i) *option[i] = (i == index);
	}

	// affinity propagation clustering
	if (clusteringDesc.affinityEnabled) {
		golem::Real affinityLambda = clusteringDesc.affinityLambda;
		golem::U32 affinitySteps = clusteringDesc.affinitySteps;
		golem::U32 affinityConvergenceSteps = clusteringDesc.affinityConvergenceSteps;
		if (desc.debugLevel >= 2 && pUICallback && pUICallback->hasInputEnabled()) {
			Menu menu(context, *pUICallback);
			menu.readNumber("Enter damping factor: ", affinityLambda);
			menu.readNumber("Enter maximum steps: ", affinitySteps);
			menu.readNumber("Enter convergence steps: ", affinityConvergenceSteps);
		}

		// similarity array
		Clustering::ValueSeqSeq similarity;
		Clustering::initialise(Size, similarity, REAL_ZERO, [=] (U32 i, U32 j) -> golem::Real { return simFunc(i, j); });

		// preferences
		Real median, deviation;
		Clustering::mad(similarity, median, deviation);
		golem::Real preferenceOffset = clusteringDesc.affinityPreferenceGain * deviation + clusteringDesc.affinityPreferenceOffset;
		if (desc.debugLevel >= 2 && pUICallback && pUICallback->hasInputEnabled()) {
			Menu menu(context, *pUICallback);
			menu.readNumber("Enter preference offset: ", preferenceOffset);
		}

		for (U32 i = 0; i < Size; ++i)
			similarity[i][i] = median + preferenceOffset;

		// affinity convergence
		auto compare = [] (const Clustering::SizeSeq& a, const Clustering::SizeSeq& b) -> bool {
			if (a.size() != b.size())
				return false;
			else for (size_t i = 0; i < a.size(); ++i)
				if (a[i] != b[i])
					return false;
			return true;
		};
		Clustering::Size step = 0;
		Clustering::SizeSeqSeq exemplarsTest(clusteringDesc.affinityConvergenceCycles);

		// affinity clustering
		Clustering::affinity(similarity, affinityLambda, affinitySteps, assignments, [&](Clustering::Size size, Clustering::Size s, const Clustering::SizeSeq& exemplars) -> bool {
			// new exemplars?
			if (!compare(exemplars, exemplarsTest[0])) {
				// debug
				if (desc.debugLevel >= 2) {
					std::string clusters("");
					for (size_t i = 0; i < exemplars.size(); ++i) {
						clusters += ", (";
						clusters += std::to_string(i + 1);
						clusters += ",";
						clusters += std::to_string(exemplars[i] + 1);
						clusters += ")";
					}
					context.debug("Aspect::clustering(): Affinity_{step=%u/%u, test=%u/%u, clusters={%u/%u%s}\n", (U32)s, (U32)affinitySteps, (U32)(step + 1), (U32)affinityConvergenceSteps, (U32)exemplars.size(), (U32)size, clusters.c_str());
				}

				// cycle?
				for (size_t i = 0; ++i < exemplarsTest.size();)
					if (compare(exemplars, exemplarsTest[i]))
						return true;

				// copy
				for (size_t i = exemplarsTest.size(); --i > 0;)
					exemplarsTest[i] = exemplarsTest[i - 1];
				exemplarsTest[0] = exemplars;

				// reset
				step = 0;
			}
			
			// finish?
			return ++step >= affinityConvergenceSteps;
		});

		//debug
		std::set<U32> clusters;
		std::for_each(assignments.begin(), assignments.end(), [&] (U32 index) { clusters.insert(index); });
		context.debug("Aspect::clustering(): Affinity_{clusters=%u, median=%f, deviation=%e, preference=%e\n", (U32)clusters.size(), median, deviation, preferenceOffset);
	}

	// PAM clustering
	if (clusteringDesc.pamEnabled) {
		golem::U32 pamSteps = clusteringDesc.pamSteps;
		golem::U32 pamClustersMin = Math::clamp((U32)(clusteringDesc.pamClustersMin*Size), U32(1), Size - 1);
		golem::U32 pamClustersMax = Math::clamp((U32)(clusteringDesc.pamClustersMax*Size), pamClustersMin + 1, Size);
		golem::Real pamFTest = clusteringDesc.pamFTest;
		if (desc.debugLevel >= 2 && pUICallback && pUICallback->hasInputEnabled()) {
			Menu menu(context, *pUICallback);
			menu.readNumber("Enter clustering steps: ", pamSteps);
			menu.readNumber("Enter min clusters: ", pamClustersMin);
			menu.readNumber("Enter max clusters: ", pamClustersMax);
			menu.readNumber("Enter cluster f-test confidence: ", pamFTest);
		}

		// distance array
		Clustering::ValueSeqSeq distance;
		Clustering::initialise(Size, distance, REAL_ZERO, [=] (U32 i, U32 j) -> golem::Real { return - simFunc(i, j); }); // - similarity

		// clustering
		const U32 clusters = Math::clamp(pamClustersMax - pamClustersMin, U32(1), Size);
		Clustering::SizeSeqSeq assignmentsSeq(clusters);
		Clustering::ValueSeq costSeq(clusters);
		for (U32 i = 0; i < clusters; ++i) {
			Clustering::pam(distance, pamClustersMin + i, pamSteps, assignmentsSeq[i], costSeq[i], rand);
			context.debug("Aspect::clustering(): PAM_{clusters=%u, cost=%f}\n", pamClustersMin + i, costSeq[i]);
		}

		// f-test
		const U32 selection = Clustering::ftest(clusters, pamFTest, [=] (U32 i) -> Real { return costSeq[i]; });
		assignments = assignmentsSeq[selection];
		context.debug("Aspect::clustering(): PAM_{selected_clusters=%u, selected_cost=%f}\n", pamClustersMin + selection, costSeq[selection]);
	}

	// No clustering
	if (none) {
		assignments.resize(Size);
		for (U32 i = 0; i < Size; ++i)
			assignments[i] = i;
	}
}

//------------------------------------------------------------------------------

void golem::Aspect::contactCluster(golem::Rand& rand, golem::U32 exemplar, const golem::U32Seq& cluster, const golem::Contact3D::Data::SeqMap& contactSeqMap, const ClusteringSim::ValueSeqSeq& simMat, golem::Contact3D::Seq& contactSeq) const {
	if (!desc.contactExemplarSubsample || cluster.empty()) {
		// default: use cluster exemplar
		contactSeq = contactSeqMap[exemplar]->second.model;
		return;
	}

	Importance::Seq importanceSeq(cluster.size() + 1);
	
	// cluster member indices with the cluster exemplar first
	importanceSeq[0].index = exemplar;
	for (U32 i = 0; i < (U32)cluster.size(); ++i)
		importanceSeq[i + 1].index = cluster[i];

	// cluster member importances/costs as distances
	Real distNorm = numeric_const<Real>::MAX;   
	for (U32 i = 0; i < (U32)importanceSeq.size(); ++i) {
		Real dist = REAL_ZERO;
		
		for (U32 j = 0; j < i; ++j)
			dist += -simMat[importanceSeq[i].index][importanceSeq[j].index].first;
		for (U32 j = i + 1; j < (U32)importanceSeq.size(); ++j)
			dist += -simMat[importanceSeq[i].index][importanceSeq[j].index].first;
		
		importanceSeq[i].importance = dist;
		if (distNorm > dist)
			distNorm = dist;
	}

	// cluster member weights
	Real sizeNorm = REAL_ZERO;
	for (U32 i = 0; i < (U32)importanceSeq.size(); ++i) {
		importanceSeq[i].weight = Math::exp( - desc.contactExemplarSubsampleDistFac*(importanceSeq[i].importance - distNorm)/distNorm);

		// cluster weighted size
		sizeNorm += importanceSeq[i].weight * contactSeqMap[importanceSeq[i].index]->second.model.size();
	}

	// normalise
	if (!golem::Sample<golem::Real>::normalise<golem::Ref1>(importanceSeq))
		throw Message(Message::LEVEL_ERROR, "Aspect::contactCluster(): unable to normalise cluster member importances");

	// re-normalise size
	const U32 clusterPoints = std::max(U32(1), (U32)Math::round(sizeNorm / importanceSeq.back().cdf));

	// debug
	if (desc.debugLevel >= 1)
		for (U32 i = 0; i < (U32)importanceSeq.size(); ++i)
			context.debug("Aspect::contactCluster(): member #%u: index=%u/%u, points=%u/%u, cost=%f, weight=%f\n", i + 1, (U32)contactSeqMap[importanceSeq[i].index]->first + 1, importanceSeq[i].index + 1, (U32)contactSeqMap[importanceSeq[i].index]->second.model.size(), clusterPoints, importanceSeq[i].importance, importanceSeq[i].weight);

	// generate contact
	contactSeq.resize(clusterPoints);
	for (U32 i = 0; i < clusterPoints; ++i) {
		// sample cluster member
		const Importance::Seq::const_iterator member = golem::Sample<golem::Real>::sample<golem::Ref1, Importance::Seq::const_iterator>(importanceSeq, rand);
		if (member == importanceSeq.end())
			throw Message(Message::LEVEL_ERROR, "Aspect::contactCluster(): unable to sample cluster member");
		const U32 index = U32(member - importanceSeq.begin());

		// sample contact
		const Contact3D::Seq::const_iterator contact = golem::Sample<golem::Real>::sample<golem::Ref1, Contact3D::Seq::const_iterator>(contactSeqMap[member->index]->second.model, rand);
		if (contact == contactSeqMap[member->index]->second.model.end())
			throw Message(Message::LEVEL_ERROR, "Aspect::contactCluster(): unable to sample contact");

		// copy
		contactSeq[i] = *contact;
		// reset weights after sampling
		contactSeq[i].weight = REAL_ONE;

		// transform
		if (index != 0) {
			const RBCoord trn(simMat[importanceSeq[0].index][importanceSeq[index].index].second); // inverse

			contactSeq[i].local.multiply(contact->local, trn);
			trn.q.multiply(contactSeq[i].projection, contact->projection);
		}
	}
}

void golem::Aspect::viewsPruning(golem::data::ContactModel::Data& data, golem::Aspect::Data& aspectData, UICallback* pUICallback) {
	// compact views and contacts
	compactData(data, aspectData);

	// if there is a collision matrix available
	if (desc.debugLevel < 3 && aspectData.viewDataMap.empty() || pUICallback && pUICallback->hasInputEnabled() && Menu(context, *pUICallback).option(0, "Aspect::viewsPruning(): compute view matrix: ", { "YES", "NO" }) == 0) {
		// re-allocate
		aspectData.viewDataMap.clear();
		Aspect::ViewData::allocate(data.views.size(), aspectData.viewDataMap);

		// memorise weights
		RealSeq weights(data.views.size());
		for (size_t i = 0; i < weights.size(); ++i)
			weights[i] = data.views[i].weight;
		// restore weights
		ScopeGuard weightsGuard([&] () {
			for (size_t i = 0; i < data.views.size(); ++i)
				data.views[i].weight = weights[i];
		});

		// reset solver state
		viewSolver->clear();
		// prepare contact model for all views
		const Contact::Map::const_iterator contact = viewSolver->add(golem::Solver::ContactTypeAny, data);

		// views collection reference
		Contact::View::Seq& views = const_cast<Contact::View::Seq&>(contact->second->getViews());
		if (views.size() != weights.size())
			throw Message(Message::LEVEL_ERROR, "Aspect::viewsPruning(): invalid view collection size %u != %u", (U32)views.size(), (U32)weights.size()); // should never happen

		// iterate over view pairs
		for (size_t i = 0, k = 0; i < data.views.size(); ++i) {
			// initialise index as 1:1
			aspectData.viewDataMap[i].index = (U32)i;

			// a single view - reset weights
			for (size_t l = 0; l < views.size(); ++l)
				views[l].weight = i == l ? REAL_ONE : REAL_ZERO;

			// compute view matrix
			for (size_t j = 0; j < data.views.size(); ++j)
				if (i != j) {
					// Point3D interface
					const data::Point3D* point3D = data.views[j].point3D;
					if (!point3D)
						throw Message(Message::LEVEL_ERROR, "Aspect::viewsPruning(): no Point3D interface provided for view #%u/%u\n", j + 1, (U32)data.views.size());

					// contact inference
					viewSolver->find(*point3D, aspectData.viewDataMap[i][j].configs);

					// evaluation
					aspectData.viewDataMap[i][j].eval = REAL_ZERO;

					// debug
					if (desc.debugLevel >= 1) {
						context.debug("Aspect::viewsPruning(): step=#%u/%u, view pair=(#%u/%u, #%u/%u), solutions=%u, eval=%u\n", ++k, (U32)data.views.size()*(data.views.size() - 1), U32(i + 1), U32(data.views.size()), U32(j + 1), U32(data.views.size()), (U32)aspectData.viewDataMap[i][j].configs.size(), aspectData.viewDataMap[i][j].eval);
					}
				}

			// evaluation
			aspectData.viewDataMap[i].eval = REAL_ZERO;
		}
	}
	else
		return;

	// pruning from Aspect::ViewData::index
	const auto pruning = [] (const Aspect::ViewData::Seq& viewData, Contact::View::Seq& views) {
		Contact::View::Seq viewsCopy;
		for (Aspect::ViewData::Seq::const_iterator i = viewData.begin(); i != viewData.end(); ++i)
			if (i->index < views.size())
				viewsCopy.push_back(views[i->index]);
		views = viewsCopy;
	};

	// optional
	if (!aspectData.procViewsPruning.empty() && pUICallback && pUICallback->hasInputEnabled() && Menu(context, *pUICallback).option(0, "Aspect::viewsPruning(): collision pruning: ", { "YES", "NO" }) == 1) {
		data.views = aspectData.procViewsPruning;
		// data.views pruning from Aspect::ViewData::index
		pruning(aspectData.viewDataMap, data.views);
		return;
	}

	// memorise initial state
	aspectData.procViewsPruning = data.views;

	// pruning based on collisions - update Aspect::ViewData::index
	// TODO

	// data.views pruning from Aspect::ViewData::index
	pruning(aspectData.viewDataMap, data.views);
}

void golem::Aspect::contactSim(golem::Rand& rand, golem::U32 i, golem::U32 j, const Contact3D::Data::SeqMap& contactSeqMap, const SimPoint::SeqSeq& pointSeqSeq, const SimVec3Seq& meanSeq, const SimRealSeq& diamSeq, Similarity& sim, std::string& msg) const {
	const U32 Size = (U32)contactSeqMap.size();
	const SimPoint::Seq& source = pointSeqSeq[i];
	const SimPoint::Seq& target = pointSeqSeq[j];

	//desc.contactSimRBDist; // CovInv ~ 1/stddev^2
	//desc.contactSimRBDistMax; // stddev
	//desc.contactSimFeatureDistMax; // stddev

	SimPoint::Dist sourceDist(desc.contactDist);
	sourceDist.setFeatureCovInv(SimPoint::Feature(contactSeqMap[i]->second.feature3DProperty.covarianceInv.begin(), contactSeqMap[i]->second.feature3DProperty.covarianceInv.end()));

	SimPoint::Dist targetDist(desc.contactDist);
	targetDist.setFeatureCovInv(SimPoint::Feature(contactSeqMap[j]->second.feature3DProperty.covarianceInv.begin(), contactSeqMap[j]->second.feature3DProperty.covarianceInv.end()));

	// no optimisiation, cloud comparison in the default frames
	if (!desc.contactSimOpt) {
		sim.first = -std::max(SimPoint::distance(sourceDist, RBCoord::identity(), source, target, false), SimPoint::distance(targetDist, RBCoord::identity(), target, source, false));
		sim.second.setId();
		return;
	}

	// DE optimisation settings
	Heuristic heuristic(context);

	//sim.second = contactSimFrame(rand, i, j, meanSeq, diamSeq);
	//RBCoord frame(sim.second);
	//SimRBCoord inverse;
	//inverse.setInverse(frame);
	//sim.first = -std::max(contactDist(frame, source, target), contactDist(inverse, target, source));
	//context.debug("Aspect::contactSim(): step=%u/%u, contact pair=(%u, %u), similarity=%e\n", k + 1, Size*(Size - 1) / 2, contactSeqMap[i]->first + 1, contactSeqMap[j]->first + 1, sim.first);
	//return;

	// sampling TODO: use more efficient (different than rejection) sampling of quaternions around given direction
	U32 trials = 0;
	heuristic.pSample = [&] (golem::Rand&, Heuristic::Vec& vec, golem::Real& value) -> bool {
		if (++trials > desc.contactSimOptInitTrials)
			throw Message(Message::LEVEL_ERROR, "Aspect::contactSim(): too many trials %u > %u: unable to generate solution candidates", trials, desc.contactSimOptInitTrials);
		vec.getRB(0).fromMat34(contactSimFrame(rand, i, j, meanSeq, diamSeq));
		value = golem::numeric_const<SimReal>::MAX;
		return false;
	};
	// preserve parameter range
	heuristic.pProcess = [=] (Heuristic::Vec& vec, Heuristic::ThreadData&) {
		vec.getRB(0).q.normalise();
	};
	// objective function
	heuristic.pValue = [&] (const Heuristic::Vec& vec, Heuristic::ThreadData&) -> Real {
		SimRBCoord inverse;
		inverse.setInverse(vec.getRB(0));
		return std::max(SimPoint::distance(sourceDist, vec.getRB(0), source, target, true), SimPoint::distance(targetDist, inverse, target, source, true));
	};
	// distance function
	heuristic.pDistance = [=] (const Heuristic::Vec& a, const Heuristic::Vec& b) -> Real {
		const Real d0 = golem::Math::sqr(a[0] - b[0]) + golem::Math::sqr(a[1] - b[1]) + golem::Math::sqr(a[2] - b[2]);
		const Real d1 = golem::numeric_const<Real>::ONE - golem::Math::abs(a[3] * b[3] + a[4] * b[4] + a[5] * b[5] + a[6] * b[6]);
		return desc.contactDist.covInv.lin*d0 + desc.contactDist.covInv.ang*d1;
	};

	// run computation
	Optimisation optimisation(heuristic);
	optimisation.start(desc.contactSimOptDesc); // throws

	// get results
	const size_t index = optimisation.stop();
	const Optimisation::Vec vec = optimisation.getVectors()[index];
	const Real val = optimisation.getValues()[index]; // objective function value

	// the final solution
	sim.first = -val;
	sim.second = vec.getRB(0).toMat34();

	// debug
	if (desc.debugLevel >= 1)
		msg = "generations=" + std::to_string(optimisation.getGenerations()) + ", trials=" + std::to_string(trials) + "/" + std::to_string(desc.contactSimOptDesc.populationSize);
}

golem::Mat34 golem::Aspect::contactSimFrame(golem::Rand& rand, golem::U32 i, golem::U32 j, const SimVec3Seq& meanSeq, const SimRealSeq& diamSeq) const {
	// Source inverse frame
	Mat34 srcTrnInv(Mat33::identity(), Vec3(meanSeq[i]));
	srcTrnInv.setInverse(srcTrnInv);

	// Target frame
	Mat34 tarTrn(Mat33::identity(), Vec3(meanSeq[j]));

	// Test randomised frame
	RBCoord testTrn;
	// Linear component
	Vec3 v;
	v.next(rand); // |v|==1
	testTrn.p.multiply(Math::abs(rand.nextGaussian<Real>(REAL_ZERO, desc.contactSimLinScale * std::max(diamSeq[i], diamSeq[j]))), v);
	// Angular component
	testTrn.q.next(rand);

	return tarTrn * testTrn.toMat34() * srcTrnInv;
}

//------------------------------------------------------------------------------

template <> void Stream::read(golem::Aspect::Data::PointsMap::value_type& value) const {
	read(const_cast<golem::U32&>(value.first));
	read(value.second, value.second.begin());
}
template <> void Stream::write(const golem::Aspect::Data::PointsMap::value_type& value) {
	write(value.first);
	write(value.second.begin(), value.second.end());
}

template <> void Stream::read(golem::Aspect::ClusteringSim::ValueSeqSeq::value_type& value) const {
	read(value, value.begin());
}
template <> void Stream::write(const golem::Aspect::ClusteringSim::ValueSeqSeq::value_type& value) {
	write(value.begin(), value.end());
}

template <> void golem::Stream::read(golem::Aspect::ViewData::Eval::Seq::value_type& value) const {
	read(value.configs, value.configs.begin());
	read(value.evals, value.evals.begin());
	read(value.eval);
}
template <> void golem::Stream::write(const golem::Aspect::ViewData::Eval::Seq::value_type& value) {
	write(value.configs.begin(), value.configs.end());
	write(value.evals.begin(), value.evals.end());
	write(value.eval);
}

template <> void Stream::read(golem::Aspect::ViewData::Seq::value_type& value) const {
	read(value.evalSeq, value.evalSeq.begin());
	read(value.eval);
	read(value.index);
}
template <> void Stream::write(const golem::Aspect::ViewData::Seq::value_type& value) {
	write(value.evalSeq.begin(), value.evalSeq.end());
	write(value.eval);
	write(value.index);
}

template <> void golem::Stream::read(golem::Aspect::Data::Map::value_type& value) const {
	read(const_cast<std::string&>(value.first));
	read(value.second.contactPartial, value.second.contactPartial.begin());
	read(value.second.contactComplete, value.second.contactComplete.begin());
	read(value.second.partialMap, value.second.partialMap.begin());
	read(value.second.completeMap, value.second.completeMap.begin());
	read(value.second.simMat, value.second.simMat.begin());

	read(value.second.procCompleteMap, value.second.procCompleteMap.begin());
	read(value.second.procContacts, value.second.procContacts.begin());
	read(value.second.procViews, value.second.procViews.begin());

	read(value.second.viewDataMap, value.second.viewDataMap.begin());

	read(value.second.pointsMap, value.second.pointsMap.begin());
	read(value.second.procViewsPruning, value.second.procViewsPruning.begin());
}
template <> void golem::Stream::write(const golem::Aspect::Data::Map::value_type& value) {
	write(value.first);
	write(value.second.contactPartial.begin(), value.second.contactPartial.end());
	write(value.second.contactComplete.begin(), value.second.contactComplete.end());
	write(value.second.partialMap.begin(), value.second.partialMap.end());
	write(value.second.completeMap.begin(), value.second.completeMap.end());
	write(value.second.simMat.begin(), value.second.simMat.end());

	write(value.second.procCompleteMap.begin(), value.second.procCompleteMap.end());
	write(value.second.procContacts.begin(), value.second.procContacts.end());
	write(value.second.procViews.begin(), value.second.procViews.end());

	write(value.second.viewDataMap.begin(), value.second.viewDataMap.end());

	write(value.second.pointsMap.begin(), value.second.pointsMap.end());
	write(value.second.procViewsPruning.begin(), value.second.procViewsPruning.end());
}

//------------------------------------------------------------------------------
