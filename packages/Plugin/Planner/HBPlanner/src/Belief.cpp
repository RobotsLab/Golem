/** @file Belief.cpp
 * 
 * @author	Claudio Zito (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include <Golem/HBPlanner/Belief.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

void golem::XMLData(Belief::Desc& val, XMLContext* context, bool create) {
	XMLData("kernels", val.kernels, context, create);
	XMLData("neighbours", val.neighbours, context, create);
	XMLData("distance_range", val.distanceRange, context, create);
	XMLData("feature_norm_eps", val.featNormEps, context, create);

	XMLData("cluster", val.cluster, context, create);

	XMLData(val.dist, context->getContextFirst("dist"), create);
	XMLData("prod", val.distProd, context->getContextFirst("dist"), create);
	XMLData("feature", val.distFeature, context->getContextFirst("dist"), create);

	XMLData(val.poseStdDev, context->getContextFirst("pose_stddev"), create);

	XMLData(&val.covariance[0], &val.covariance[RBCoord::N], "c", context->getContextFirst("covariance"), create);

	XMLData("population_size", val.populationSize, context->getContextFirst("mean_shift"), create);
	XMLData("generations_min", val.generationsMin, context->getContextFirst("mean_shift"), create);
	XMLData("generations_max", val.generationsMax, context->getContextFirst("mean_shift"), create);
	XMLData("distance_diff", val.distanceDiff, context->getContextFirst("mean_shift"), create);

	try {
		val.localEnabled = true;
		val.optimisationDesc.load(context->getContextFirst("optimisation"));
	}
	catch (const Message&) {
		val.localEnabled = false;
	}
	XMLData("distance_max", val.distanceMax, context, create);
	XMLData(*val.hypothesisDescPtr, context->getContextFirst("hypothesis"), create);
	try {
		XMLData((RBPose::Desc&)*val.rbPoseDescPtr, context->getContextFirst("pose_estimation"));
	}
	catch (const MsgXMLParser&) {}

	//XMLData("kernels", val.tactile.kernels, context, create);
	//XMLData("test", val.tactile.test, context, create);
	//XMLData(val.tactile.stddev, context->getContextFirst("test_stddev"), create);
	//XMLData(&val.tactile.covariance[0], &val.tactile.covariance[RBCoord::N], "c", context->getContextFirst("covariance"), create);
	XMLData("num_hypotheses", val.numHypotheses, context->getContextFirst("tactile_model"), create);
	XMLData("max_surface_points", val.maxSurfacePoints, context->getContextFirst("tactile_model"), create);
}

template <> void Stream::read(Belief& belief) const {
	belief.getSamples().clear();
	read(belief.getSamples(), belief.getSamples().begin());
	belief.getHypotheses().clear();
	read(belief.getHypotheses(), belief.getHypotheses().begin());
}

template <> void Stream::write(const Belief& belief) {
	write(belief.getSamples().begin(), belief.getSamples().end());
	write(belief.getHypotheses().begin(), belief.getHypotheses().end());
}

//------------------------------------------------------------------------------

golem::Belief::Ptr golem::Belief::Desc::create(Context &context) const {
	Belief::Ptr belief(new Belief(context));
	belief->create(*this);
	return belief;
}

//------------------------------------------------------------------------------

golem::Belief::Belief(Context& context) : context(context), rand(context.getRandSeed()){
//	pRBPose.reset((RBPose*)this);
}

//Belief::~Belief() {
//}


bool golem::Belief::create(const Desc& desc) {
	desc.assertValid(Assert::Context("Belief::Desc."));
	
	this->desc = desc;

	rbPosePtr = desc.rbPoseDescPtr->create(context);

	appearance.setToDefault();

	poses.clear();
	poses.resize(this->desc.kernels);

	// reset container for the low-dim rep belief
	hypotheses.clear();
	hypotheses.reserve(this->desc.numHypotheses);

	this->desc.poseStdDev.ang = Math::sqrt(REAL_ONE / this->desc.poseStdDev.ang);	// stdDev ~ 1/cov
	poseCovInv.lin = REAL_ONE / (poseCov.lin = Math::sqr(this->desc.poseStdDev.lin));
	poseCovInv.ang = REAL_ONE / (poseCov.ang = Math::sqr(this->desc.poseStdDev.ang));

	manipulator.reset();

	normaliseFac = REAL_ZERO;

	realPose.setId();

	context.write("Belief::create(): kernels %d\nRBPose::create(): kernels %d, features %d\n", desc.kernels, desc.rbPoseDescPtr->kernels, desc.rbPoseDescPtr->features);
//	mfsePoses.resize(desc.tactile.kernels);

	return true;
}

//------------------------------------------------------------------------------

void golem::Belief::drawSamples(const size_t numSamples, DebugRenderer& renderer) const {
	for (size_t t = 0; t < numSamples; ++t) {
		RBCoord s = sample();

		if (appearance.showFrames)
			renderer.addAxes(s.toMat34() * modelFrame, appearance.frameSize);

		if (appearance.showPoints) {
			Cloud::PointSeq seq;
			for (Cloud::PointSeq::const_iterator i = modelPoints.begin(); i != modelPoints.end(); ++i) {
				Cloud::Point point = *i;
				Cloud::setColour(appearance.colour, point);
				seq.push_back(point);
			}
			Cloud::transform(s.toMat34(), seq, seq);
			for (Cloud::PointSeq::const_iterator i = seq.begin(); i != seq.end(); ++i)
				renderer.addPoint(Cloud::getPoint<Real>(*i));
		}
	}
}

void golem::Belief::drawHypotheses(DebugRenderer &renderer, const bool showOnlyMeanPose) const {
	context.write("Belief::drawHypotheses (%s)\n", showOnlyMeanPose ? "ON" : "OFF");
	//for (Hypothesis::Seq::const_iterator h = hypotheses.begin(); h != hypotheses.end(); ++h) {
	//	(*h)->draw(renderer);
	//	if (showOnlyMeanPose)
	//		return;
	//}
}

golem::Bounds::Seq golem::Belief::uncertaintyRegionBounds() {
	Bounds::Seq bounds;
	bounds.push_back(uncertaintyDesc.create());
	return bounds;
}

//------------------------------------------------------------------------------

void golem::Belief::set(const RBPose::Sample::Seq &poseSeq, const RBPose::Sample::Seq &hypothesisSeq, const Mat34 &trn, const Cloud::PointSeq &points) {
	modelPoints.clear();
	modelPoints.reserve(points.size());
	for (Cloud::PointSeq::const_iterator i = points.begin(); i != points.end(); ++i)
		modelPoints.push_back(*i);
	context.write("Belief::set(): model points size %d\n", modelPoints.size());
	modelFrame = trn;

	try {
		setPoses(poseSeq);
		setHypotheses(hypothesisSeq);
	}
	catch (const Message &msg) {
		context.getMessageStream()->write(msg);
		context.write("\n");
	}
}

void golem::Belief::setPoses(const RBPose::Sample::Seq &poseSeq) {
	if (poseSeq.empty())
		throw Message(Message::LEVEL_ERROR, "Belief::setPoses(): Invalid samples.");

	// reset containers
	poses.clear();
	poses.reserve(poseSeq.size());
	initPoses.clear();
	initPoses.reserve(poseSeq.size());

	// copy items
	for (RBPose::Sample::Seq::const_iterator p = poseSeq.begin(); p != poseSeq.end(); ++p) {
		poses.push_back(*p);
		initPoses.push_back(*p);
	}

	// mean and covariance
	if (!pose.create<Ref1, RBPose::Sample::Ref>(RBCoord::N, desc.covariance, poses))
		throw Message(Message::LEVEL_ERROR, "RBPose::createQuery(): Unable to create mean and covariance for the high dimensional representation");
	// copy initial distribution properties
	initProperties = pose;


	context.write("Belief::setPoses(): covariance mfse = {(%f, %f, %f), (%f, %f, %f, %f)}\n", initProperties.covariance[0], initProperties.covariance[1], initProperties.covariance[2], initProperties.covariance[3], initProperties.covariance[4], initProperties.covariance[5], initProperties.covariance[6]);
}

/** Sets hypotheses */
void golem::Belief::setHypotheses(const RBPose::Sample::Seq &hypothesisSeq) {
	if (hypothesisSeq.empty() || !modelFrame.isFinite() || modelPoints.empty())
		throw Message(Message::LEVEL_ERROR, "Belief::setHypotheses(): Invalid samples (Hypotheses size %d, modelFrame is %s, modelPoints size %d)", hypothesisSeq.size(), modelFrame.isFinite()?"finite":"not finite", modelPoints.size());

	context.debug("----------------------------------------------------------\n");
	context.debug("Belif:setHypotheses()\n");

	// reset container for hypotheses
	hypotheses.clear();
	hypotheses.resize(hypothesisSeq.size());
	U32 idx = 0;
	Hypothesis::Seq::iterator i = hypotheses.begin();
	for (RBPose::Sample::Seq::const_iterator p = hypothesisSeq.begin(); p != hypothesisSeq.end(); ++p, ++i) {
		// container for the point cloud of this sample (default: the same points of the model)
//		RBPose::Sample sample(*p);
		Cloud::PointSeq sampleCloud;
		const size_t size = modelPoints.size() < desc.maxSurfacePoints ? modelPoints.size() : desc.maxSurfacePoints;
		for (size_t j = 0; j < size; ++j) {
			Cloud::Point point = size < modelPoints.size() ? modelPoints[size_t(rand.next()) % modelPoints.size()] : modelPoints[j]; // make a copy here
			Cloud::setPoint(p->toMat34() * Cloud::getPoint<Real>(point)/* + actionFrame.p*/, point);
			Cloud::setColour((p == hypothesisSeq.begin()) ? RGBA::GREEN : RGBA::BLUE, point);
			sampleCloud.push_back(point);
		}
//		hypotheses.push_back(Hypothesis::Ptr(new Hypothesis(idx, modelFrame, *p, sampleCloud)));
		(*i) = desc.hypothesisDescPtr->create(*manipulator.get());
		(*i)->create(idx, modelFrame, *p, rand, sampleCloud);
		(*i)->appearance.colour = (p == hypothesisSeq.begin()) ? RGBA::GREEN : RGBA::BLUE;
		(*i)->appearance.showPoints = true;
		context.debug("Hypothesis n.%d <%.4f %.4f %.4f> <%.4f %.4f %.4f %.4f>\n", idx, p->p.x, p->p.y, p->p.z, p->q.w, p->q.x, p->q.y, p->q.z);
		idx++;
	}

	// mean and covariance
	RBPose::Sample::Seq seq = getHypothesesToSample();
	if (!sampleProperties.create<Ref1, RBPose::Sample::Ref>(RBCoord::N, desc.covariance, seq))
		throw Message(Message::LEVEL_ERROR, "RBPose::createQuery(): Unable to create mean and covariance for the high dimensional representation");

	// compute a volumetric region of the uncertainty (min 10 cms in each direction)
	Vec3 cov(pose.covariance[0], pose.covariance[1], pose.covariance[2]*100);
	uncertaintyDesc.dimensions = hypotheses.front()->boundsDesc.dimensions + cov*1000;//Vec3(pose.covariance[0] * 5000 > d ? pose.covariance[0] * 5000 : d, pose.covariance[1] * 5000 > d ? pose.covariance[1] * 5000 : d, pose.covariance[2] * 5000 > d ? pose.covariance[2] * 5000 : d);
	context.write("Uncertainty dimensions = [%f, %f, %f]\n", uncertaintyDesc.dimensions.x, uncertaintyDesc.dimensions.y, uncertaintyDesc.dimensions.z);
	uncertaintyDesc.pose.p = hypotheses.front()->toRBPoseSampleGF().p;

	context.debug("Sub-sampled covariance = {(%f, %f, %f), (%f, %f, %f, %f)}\n", sampleProperties.covariance[0], sampleProperties.covariance[1], sampleProperties.covariance[2], sampleProperties.covariance[3], sampleProperties.covariance[4], sampleProperties.covariance[5], sampleProperties.covariance[6]);
}


golem::RBPose::Sample golem::Belief::createHypotheses(const Cloud::PointSeq& model, const Mat34 &transform/*, const bool init*/) {
	U32 idx = 0;
	// copy model and model frame. Note: it is done only the very first time.
	if (modelPoints.empty()) {
		modelPoints.clear();
		modelPoints.resize(model.size());
		for (Cloud::PointSeq::const_iterator i = model.begin(); i != model.end(); ++i) 
			modelPoints.push_back(*i);
		modelFrame = transform;
	}
	// reset container for hypotheses
	hypotheses.clear();
	hypotheses.resize(desc.numHypotheses);

	//const RBPose::Sample maximumFrame = maximum();
	RBPose::Sample bestSample;
	RBPose::Sample::Seq clusters;
	meanShiftClustering(bestSample, clusters);
	
	//Vec3 offset(-.1, .07, .0);
	//RBPose::Sample actionFrame = maximum();
	//	context.write("Heuristic:setBeliefState(model size = %d, max points = %d): samples: cont_fac = %f\n", model.size(), ftDrivenDesc.maxSurfacePoints, ftDrivenDesc.contactFac);
	context.debug("----------------------------------------------------------\n");
	context.debug("Belief:createHypotheses()\n");
	U32 clusterId = 0;
	for (Hypothesis::Seq::iterator i = hypotheses.begin(); i < hypotheses.end(); ++i) {
		// sample hypothesis. NOTE: The first element is the max scoring pose
		//RBCoord actionFrame = (i == hypotheses.begin()) ? maximumFrame : sample();
		const RBCoord actionFrame = (i == hypotheses.begin()) ? bestSample : [&]() -> const RBCoord {
			if (clusterId < clusters.size()) return clusters[clusterId++];
			else return sample();
		}();

		//sampleFrame.p += offset;
		// transform the sample in the reference coordinate (default: robot's coordinate frame)
//		sampleFrame.multiply(sampleFrame, RBCoord(transform));
		// container for the point cloud of this sample (default: the same points of the model)
		
		// copy model point cloud
		// optimisation: limit the number of points to save performance in the kd-tree
		const size_t size = model.size() < desc.maxSurfacePoints ? model.size() : desc.maxSurfacePoints;
		Cloud::PointSeq sampleCloud;
		sampleCloud.resize(size);
		for (size_t j = 0; j < size; ++j) {
			Cloud::Point p = size < model.size() ? model[size_t(rand.next())%model.size()] : model[j]; // make a copy here
			Cloud::setPoint(actionFrame.toMat34() * Cloud::getPoint<Real>(p)/* + actionFrame.p*/, p);
			Cloud::setColour((i == hypotheses.begin()) ? RGBA::GREEN : RGBA::BLUE, p);
			sampleCloud[j] = p;
		}
		(*i) = desc.hypothesisDescPtr->create(*manipulator);
		(*i)->create(idx, modelFrame, RBPose::Sample(actionFrame), rand, sampleCloud);
		(*i)->appearance.colour = (i == hypotheses.begin()) ? RGBA::GREEN : RGBA::BLUE;
		(*i)->appearance.showPoints = true;
		//Mat34 sampleFrame; 
		//sampleFrame.multiply(actionFrame.toMat34(), modelFrame);
		//RBCoord sf(sampleFrame);
		context.write("Hypothesis %d { %.5f\t%.5f\t%.5f\t%.5f\t%.5f\t%.5f\t%.5f }\n", idx, actionFrame.p.x, actionFrame.p.y, actionFrame.p.z, actionFrame.q.w, actionFrame.q.x, actionFrame.q.y, actionFrame.q.z);
		//context.write("HypothesisGF %d { %.5f\t%.5f\t%.5f\t%.5f\t%.5f\t%.5f\t%.5f }\n", idx, sf.p.x, sf.p.y, sf.p.z, sf.q.w, sf.q.x, sf.q.y, sf.q.z);
		idx++;
	}
	
	// mean and covariance
	RBPose::Sample::Seq seq = getHypothesesToSample();
	if (!sampleProperties.create<Ref1, RBPose::Sample::Ref>(RBCoord::N, desc.covariance, seq))
		throw Message(Message::LEVEL_ERROR, "RBPose::createHypotheses(): Unable to create mean and covariance for the high dimensional representation");
		
	context.write("Sub-sampled covariance = {(%f, %f, %f), (%f, %f, %f, %f)}\n", sampleProperties.covariance[0], sampleProperties.covariance[1], sampleProperties.covariance[2], sampleProperties.covariance[3], sampleProperties.covariance[4], sampleProperties.covariance[5], sampleProperties.covariance[6]);
	
	// compute determinant for the sampled hypotheses
	covarianceDet = REAL_ONE;
	for (U32 j = 0; j < RBCoord::N; ++j)
			covarianceDet *= sampleProperties.covariance[j];

	// compute a volumetric region of the uncertainty (min 10 cms in each direction)
	Vec3 cov(pose.covariance[0], pose.covariance[1], pose.covariance[2] * 100);
	uncertaintyDesc.dimensions = hypotheses.front()->boundsDesc.dimensions + cov * 1000;//Vec3(pose.covariance[0] * 5000 > d ? pose.covariance[0] * 5000 : d, pose.covariance[1] * 5000 > d ? pose.covariance[1] * 5000 : d, pose.covariance[2] * 5000 > d ? pose.covariance[2] * 5000 : d);
	uncertaintyDesc.pose.p = hypotheses.front()->toRBPoseSampleGF().p;

	//// Reduce the number of poses to reduce the computational time of belief update
	//if (init) {
	//	RBPose::Sample::Seq tmp;
	//	for (size_t i = 0; i < myDesc.numPoses; ++i)
	//		tmp.push_back(sample());
	//	poses.clear();
	//	poses.reserve(myDesc.numPoses);
	//	for (size_t i = 0; i < myDesc.numPoses; ++i) {
	//		poses.push_back(RBPose::Sample(RBCoord(tmp[i].p, tmp[i].q), REAL_ONE, i*REAL_ONE));
	//	}
	//
	//	// set initial belief
	//	initPoses.clear();
	//	initPoses.reserve(myDesc.numPoses);
	//	for (RBPose::Sample::Seq::const_iterator i = poses.begin(); i != poses.end(); ++i) {
	//		initPoses.push_back(*i);
	//	}
	//	initProperties = sampleProperties;
	//}
	//actionFrame.p += offset;
	return bestSample; // maximumFrame;
}

golem::RBPose::Sample::Seq golem::Belief::getHypothesesToSample() const {
	// return the rbpose::sample associated with each hypothesis.
	// NOTE: useful for creating mean and covariance in Belief::setHypotheses
	RBPose::Sample::Seq samples;
	for (Hypothesis::Seq::const_iterator h = hypotheses.begin(); h != hypotheses.end(); ++h)
		samples.push_back((*h)->toRBPoseSample());
	return samples;
}

//------------------------------------------------------------------------------

void golem::Belief::createQuery(const Cloud::PointSeq& points) {
	SecTmReal init = context.getTimer().elapsed();
	poses.clear();
	poses.resize(desc.kernels);
	for (size_t i = 0; i < desc.kernels; ++i) {
		context.write("Belief::Query: %d/%d\r", i, desc.kernels);
		rbPosePtr->createQuery(points);
		poses[i] = rbPosePtr->maximum();
		//context.debug("%.5f\t%.5f\t%.5f\t%.5f\t%.5f\t%.5f\t%.5f\n", s.p.x, s.p.y, s.p.z, s.q.w, s.q.x, s.q.y, s.q.z);
		context.debug("----------------------------------------------------------\n");
	}
	//	CriticalSection cs;
	//	size_t i = 0;
	//	ParallelsTask(context.getParallels(), [&](ParallelsTask*) {
	//		const U32 jobId = context.getParallels()->getCurrentJob()->getJobId();
	//		if (jobId > U32(4))
	//			return;
	//		size_t j = i;
	//		RBPose::Ptr ptr = desc.rbPoseDescPtr->create(context);
	//		ptr.reset(new RBPose(*rbPosePtr.get()));
	//		for (;;) {
	//			{
	//				CriticalSectionWrapper csw(cs);
	//				j = i++;
	//				if (j >= desc.kernels)
	//					break;
	//			}
	//			context.write("Belief::Query: %d/%d\r", j + 1, desc.kernels);
	//			ptr->createQuery(points);
	//			poses[j] = ptr->maximum();
	//			//		RBCoord c; c.multiply(s, modelFrame);
	//			//context.debug("%.5f\t%.5f\t%.5f\t%.5f\t%.5f\t%.5f\t%.5f\n", s.p.x, s.p.y, s.p.z, s.q.w, s.q.x, s.q.y, s.q.z);
	//			//poses[j] = s;
	//			//context.debug("----------------------------------------------------------\n");
	//		}
	//});
	const SecTmReal t_end = context.getTimer().elapsed() - init;
	
	// mean and covariance
	if (!pose.create<Ref1, RBPose::Sample::Ref>(RBCoord::N, desc.covariance, poses))
		throw Message(Message::LEVEL_ERROR, "Belief::createQuery(): Unable to create mean and covariance for the high dimensional representation");
	
	// generate new (noisy) samples out of selected subset of poses 
	for (size_t i = 0; i < desc.kernels; ++i)
		rand.nextGaussianArray<Real>(&(poses[i])[0], &(poses[i])[0] + RBCoord::N, &((poses[i]))[0], &pose.covarianceSqrt[0]); // normalised multivariate Gaussian

	context.write("Belief::createQuery(): elapsed_time=%f\n", t_end);
	context.write("Belief::createQuery(): sampled covariance = {(%f, %f, %f), (%f, %f, %f, %f)}\n",
		pose.covariance[0], pose.covariance[1], pose.covariance[2], pose.covariance[3],
		pose.covariance[4], pose.covariance[5], pose.covariance[6]);
}

golem::Real golem::Belief::maxWeight(const bool normalised) const {
	Real max = REAL_ZERO;
	for (RBPose::Sample::Seq::const_iterator s = poses.begin(); s != poses.end(); ++s) {
		const Real w = normalised && normaliseFac > REAL_ZERO ? s->weight / normaliseFac : s->weight;
		if (w > max)
			max = w;
	}
	return max;
}

void golem::Belief::meanShiftClustering(RBPose::Sample& solution, RBPose::Sample::Seq& clusters) {
	context.debug("RBPose::alignGlobal(): Global alignment...\n");
	struct Cluster {
		RBPose::Sample sample;
		Real eval;
		U32 votes;
	};
	typedef std::map<U32, Cluster> ClusterMap;
	
	ClusterMap clusterMap;
	U32 numClusters = 0, totalVotes = 0;
	clusters.clear();
	
	for (size_t i = 0; i < 10; ++i) {
		RBPose::Sample sol;
		U32 votes = 0;
		Real solutionEval = REAL_MIN;
		alignGlobal(sol, solutionEval, votes);
		totalVotes += votes;
		context.write("align global [eval votes] = [%f %u/%u]\n", solutionEval, votes, totalVotes);

		ClusterMap::iterator ptr = clusterMap.end();
		Real min = REAL_MAX;
		for (ClusterMap::iterator j = clusterMap.begin(); j != clusterMap.end(); ++j) {
			const Real d = distance(sol, j->second.sample);
			if (d < desc.distanceRange && d < min) {
				ptr = j;
				min = d;
			}
		}
		if (ptr != clusterMap.end()) {
			RBCoord mean(Vec3(REAL_ZERO), Quat(REAL_ZERO, REAL_ZERO, REAL_ZERO, REAL_ZERO)), meanBuf(Vec3(REAL_ZERO), Quat(REAL_ZERO, REAL_ZERO, REAL_ZERO, REAL_ZERO));
			Real norm = REAL_ZERO, normBuf = REAL_ZERO;
			const Real w = solutionEval*kernel(min);
			kahanSum(norm, normBuf, w);
			kahanSum(norm, normBuf, ptr->second.eval);
			for (size_t l = 0; l < RBCoord::N; ++l)
				kahanSum(mean[l], meanBuf[l], ptr->second.eval*ptr->second.sample[l]);
			for (size_t l = 0; l < RBCoord::N; ++l)
				kahanSum(mean[l], meanBuf[l], w*sol[l]);
			
			if (norm <= REAL_ZERO)
				break;
			
			const Real normInv = REAL_ONE / norm;
			mean.p *= normInv;
			mean.q *= normInv;
			mean.q.normalise();
			ptr->second.sample = mean;
			ptr->second.eval = norm / 2;
			ptr->second.votes += votes;
		}
		else {
			Cluster cluster; cluster.sample = sol; cluster.eval = solutionEval; cluster.votes = votes;
			clusterMap.insert(clusterMap.end(), ClusterMap::value_type(numClusters++, cluster));
		}
	}

	if (clusterMap.empty())
		throw Message(Message::LEVEL_ERROR, "Belief::meanShiftClustering(): no clusters");

	ClusterMap::iterator best = clusterMap.end();
	Real bestEval = REAL_ZERO;
	U32 voteMax = 0;
	context.write("Sim obj [%.2f %.2f %.2f %.2f %.2f %.2f %.2f]\n", realPose.p.x, realPose.p.y, realPose.p.z, realPose.q.w, realPose.q.x, realPose.q.y, realPose.q.z);

	for (ClusterMap::iterator j = clusterMap.begin(); j != clusterMap.end(); ++j) {
		const Real score = (j->second.votes / Real(totalVotes)) * j->second.eval;
		RBCoord s(j->second.sample * modelFrame);
		const RBDist d(realPose, RBCoord(j->second.sample * modelFrame));
		context.write("cluster [%u] eval=%.5f votes=%u score=%.5f error lin=%.5f ang=%.5f\n  mean [%.2f %.2f %.2f %.2f %.2f %.2f %.2f]\n", j->first, j->second.eval, j->second.votes, score, Math::sqrt(d.lin), d.ang,
			s.p.x, s.p.y, s.p.z, s.q.w, s.q.x, s.q.y, s.q.z);

		if (/*j->second.eval*/ score > bestEval) {
			best = j;
			bestEval = score;// j->second.eval;
			voteMax = j->second.votes;
		}
	}
	context.write("BEST: cluster [%u]\n", best->first);

	solution = best->second.sample;
	for (ClusterMap::iterator j = clusterMap.begin(); j != clusterMap.end(); ++j)
		if (j->first != best->first)
			clusters.push_back(j->second.sample);
}

void golem::Belief::alignGlobal(RBPose::Sample& solution, Real& solutionEval, U32& votes) {
	context.debug("RBPose::alignGlobal(): Global alignment...\n");

	size_t k = 0;
	solutionEval = REAL_MIN;
	U32 thisClusterVotes = 0;
	CriticalSection cs;
	ParallelsTask(context.getParallels(), [&](ParallelsTask*) {
		RBCoord test;
		Real testEval = REAL_MIN;
		for (;;) {
			{
				CriticalSectionWrapper csw(cs);
				if (solutionEval < testEval) {
					solutionEval = testEval;
					solution = test;
					votes = thisClusterVotes;
				}
				if (++k > desc.populationSize)
					break;
			}

			test = sample();
			thisClusterVotes = 1;
			for (size_t j = 0; j < desc.generationsMax; ++j) {
				// approximate affine combination of quaternions: renormalisation
				RBCoord mean(Vec3(REAL_ZERO), Quat(REAL_ZERO, REAL_ZERO, REAL_ZERO, REAL_ZERO)), meanBuf(Vec3(REAL_ZERO), Quat(REAL_ZERO, REAL_ZERO, REAL_ZERO, REAL_ZERO));
				Real norm = REAL_ZERO, normBuf = REAL_ZERO;
				for (size_t i = 0; i < desc.neighbours; ++i) {
					const RBPose::Sample s = poses[rand.next() % poses.size()];
					const Real d = distance(test, s);
					if (d < desc.distanceRange) {
						++thisClusterVotes;
						const Real w = s.weight*kernel(d);
						kahanSum(norm, normBuf, w);
						for (size_t l = 0; l < RBCoord::N; ++l)
							kahanSum(mean[l], meanBuf[l], w*s[l]);
					}
				}
				if (norm <= REAL_ZERO)
					break;
				const Real normInv = REAL_ONE / norm;
				mean.p *= normInv;
				mean.q *= normInv;
				mean.q.normalise();
				const Real d = distance(test, mean);
				test = mean;
				if (j > desc.generationsMin && d < desc.distanceDiff)
					break;
			}

			testEval = density(test);
		}
	});

	// print debug message
	context.debug("RBPose::alignGlobal(): Objective value: %f\n", solutionEval);
}

golem::RBPose::Sample golem::Belief::maximum() {
	RBPose::Sample solution;
	RBPose::Sample::Seq clusters;
	U32 votes;
	Real solutionEval = REAL_ZERO;
	if (desc.cluster)
		meanShiftClustering(solution, clusters);
	else
		alignGlobal(solution, solutionEval, votes);
	return solution;
}

void golem::Belief::createResample(/*const Manipulator::Config& robotPose*/) {
	// check for collisions to reject samples
	//HBCollision::FlannDesc waypointDesc;
	//HBCollision::Desc::Ptr cloudDesc;
	//cloudDesc.reset(new HBCollision::Desc());
	//HBCollision::Ptr cloud = cloudDesc->create(*manipulator);
	//waypointDesc.depthStdDev = 0.0005; waypointDesc.likelihood = 1000.0; waypointDesc.points = 10000; waypointDesc.neighbours = 100;
	//waypointDesc.radius = -0.005;
	 
	size_t N = poses.size(), index = rand.nextUniform<size_t>(0, N);
	Real beta = REAL_ZERO;
	RBPose::Sample::Seq newPoses;
	newPoses.reserve(N);
	for (size_t i = 0; i < N; ++i) {
		beta += rand.nextUniform<Real>()*2*maxWeight();
//		context.write("RBPose::createResampling(): beta=%4.6f\n", beta);
		while (beta > poses[index].weight) {
			beta -= poses[index].weight;
			index = (index + 1) % N;
//			context.write("beta=%4.6f\n, index=%d, weight=%4.6f\n", beta, index, poses[index].weight);
		}
		newPoses.push_back(poses.at(index));
	}
	
	// add noise to the resampled elements and overwrite poses
	poses.clear();
	poses.reserve(N);

	// generate new (noisy) samples out of selected subset of poses 
	for (size_t i = 0; i < N; ++i) {
		//mfsePoses.push_back(Sample(newPoses[i], REAL_ONE, i*REAL_ONE));
		//continue;
		RBCoord c = newPoses[i];
		rand.nextGaussianArray<Real>(&c[0], &c[0] + RBCoord::N, &(newPoses[i])[0], &pose.covarianceSqrt[0]); // normalised multivariate Gaussian
		//Cloud::PointSeq points;
		//Cloud::transform(c.toMat34(), modelPoints, points);
		//cloud->create(rand, points);
		//if (cloud->check(waypointDesc, rand, robotPose)) continue;
		poses.push_back(RBPose::Sample(c, REAL_ONE, i*REAL_ONE));
		//++i;
	}
	normaliseFac = REAL_ZERO;
	
	// compute mean and covariance
	if (!pose.create<Ref1, RBPose::Sample::Ref>(RBCoord::N, desc.covariance, poses))
		throw Message(Message::LEVEL_ERROR, "RBPose::createResample(): Unable to create mean and covariance for the high dimensional representation");

	context.write("Belief::createResample(): covariance mfse = {(%f, %f, %f), (%f, %f, %f, %f)}\n", pose.covariance[0], pose.covariance[1], pose.covariance[2], pose.covariance[3], pose.covariance[4], pose.covariance[5], pose.covariance[6]);
}

//------------------------------------------------------------------------------

golem::RBCoord golem::Belief::sample() const {
	RBPose::Sample::Seq::const_iterator ptr = RBPose::Sample::sample<Ref1, RBPose::Sample::Seq::const_iterator>(poses, rand);
	if (ptr == poses.end())
		throw Message(Message::LEVEL_ERROR, "RBPose::sample(): Sampling error");

	RBCoord c;
	//Vec3 v;
	//v.next(rand); // |v|==1
	//v.multiply(Math::abs(rand.nextGaussian<Real>(REAL_ZERO, desc.poseStdDev.lin)), v);
	//c.p.add(ptr->p, v);
	//Quat q;
	//q.next(rand, poseCovInv.ang);
	//c.q.multiply(ptr->q, q);
	this->rand.nextGaussianArray<Real>(&c[0], &c[0] + RBCoord::N, &(*ptr)[0], &pose.covarianceSqrt[0]); // normalised multivariate Gaussian
	//RBDist noise(*ptr, c);
	//context.write("Sampling noise [lin ang] = [%f %f]\n", noise.lin, noise.ang);
	return c;
}

golem::Real golem::Belief::distance(const RBCoord& a, const RBCoord& b) const {
	//return poseCovInv.dot(RBDist(a, b));

	const Real d0 = pose.covarianceInv[0] * Math::sqr(a[0] - b[0]) + pose.covarianceInv[1] * Math::sqr(a[1] - b[1]) + pose.covarianceInv[2] * Math::sqr(a[2] - b[2]);
	const Real d1 = pose.covarianceInv[3] * Math::sqr(a[3] - b[3]) + pose.covarianceInv[4] * Math::sqr(a[4] - b[4]) + pose.covarianceInv[5] * Math::sqr(a[5] - b[5]) + pose.covarianceInv[6] * Math::sqr(a[6] - b[6]);
	const Real d2 = pose.covarianceInv[3] * Math::sqr(a[3] + b[3]) + pose.covarianceInv[4] * Math::sqr(a[4] + b[4]) + pose.covarianceInv[5] * Math::sqr(a[5] + b[5]) + pose.covarianceInv[6] * Math::sqr(a[6] + b[6]);
	return REAL_HALF*(d0 + std::min(d1, d2));
}

golem::Real golem::Belief::kernel(Real distance) const {
	return Math::exp(-distance); // exponential
}

golem::Real golem::Belief::density(const RBCoord &c) const {
	Real sum = REAL_ZERO;
	Real buf = REAL_ZERO;
	for (RBPose::Sample::Seq::const_iterator i = poses.begin(); i != poses.end(); ++i) {
		const Real dist = distance(c, *i);
		if (dist < desc.distanceRange)
			kahanSum(sum, buf, i->weight*kernel(dist));
	}

	return sum;// up to scaling factor
}

void golem::Belief::createUpdate(DebugRenderer& renderer, const Waypoint &w, FTGuard::SeqPtr& triggeredGuards) {
	HBCollision::FlannDesc waypointDesc;
	HBCollision::Desc::Ptr cloudDesc;
	cloudDesc.reset(new HBCollision::Desc());
	HBCollision::Ptr cloud = cloudDesc->create(*manipulator);
	waypointDesc.depthStdDev = 0.01/*0.0005*/; waypointDesc.likelihood = 1000.0; waypointDesc.points = 10000; waypointDesc.neighbours = 100;
	waypointDesc.radius = REAL_ZERO;
	
	Real norm = REAL_ZERO, c = REAL_ZERO, cdf = REAL_ZERO;
	Manipulator::Config config(w.cpos, manipulator->getBaseFrame(w.cpos));
	normaliseFac = REAL_ZERO;
	for (RBPose::Sample::Seq::iterator sampledPose = poses.begin(); sampledPose != poses.end(); ++sampledPose) {
		Cloud::PointSeq points;
		Cloud::transform(sampledPose->toMat34(), modelPoints, points);
		//renderer.reset();
		//debugAppearance.draw(points, renderer);
		cloud->create(rand, points);
		sampledPose->weight = cloud->evaluateFT(renderer, waypointDesc, config, triggeredGuards, false);

		// sum weights
		kahanSum(normaliseFac/*norm*/, c, sampledPose->weight/*sampledPose->weight > 0 ? sampledPose->weight : Math::log10(REAL_EPS)*/);
	}

	// normalise weights
	c = REAL_ZERO;
	for (RBPose::Sample::Seq::iterator sampledPose = poses.begin(); sampledPose != poses.end(); ++sampledPose) {
		kahanSum(cdf, c, sampledPose->weight);
		sampledPose->cdf = cdf;
	}
}
