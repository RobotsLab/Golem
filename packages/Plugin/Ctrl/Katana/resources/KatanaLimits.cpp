#include <stdlib.h>
#include <stdio.h>
#include <Golem/Math/Queue.h>
#include <Golem/Math/Math.h>
#include <Golem/Defs/Assert.h>

using namespace golem;


Real posFromEnc(int pos, Real angleOffset, int encoderOffset, int encodersPerCycle, int rotationDirection, Real offset, Real gain) {
	return offset + gain*(angleOffset - REAL_2_PI*(pos - encoderOffset)/Real(encodersPerCycle*rotationDirection));
}
int posToEnc(Real pos, Real angleOffset, int encoderOffset, int encodersPerCycle, int rotationDirection, Real offset, Real gain) {
	return (int)Math::round(Real(encoderOffset) + (angleOffset - (pos - offset)/gain)*Real(encodersPerCycle*rotationDirection)/REAL_2_PI);
}

void test(int id, Real angleOffset, Real angleRange, int encoderOffset, int encodersPerCycle, int rotationDirection, int encoderPositionAfter, Real offset, Real gain) {
	printf("\nAXIS #%d:\nangleOffset=%f, angleRange=%f, encoderOffset=%i, encodersPerCycle=%i, rotationDir=%i, encoderPositionAfter=%i, offset=%f, gain=%f\n",
		id, angleOffset, angleRange, encoderOffset, encodersPerCycle, rotationDirection, encoderPositionAfter, offset, gain);
	printf("angle (encoder min) = %f\n",
		posFromEnc(encoderOffset, angleOffset*REAL_2_PI/360.0, encoderOffset, encodersPerCycle, rotationDirection, offset, gain));
	printf("angle (encoder init) = %f\n", 
		posFromEnc(encoderPositionAfter, angleOffset*REAL_2_PI/360.0, encoderOffset, encodersPerCycle, rotationDirection, offset, gain));

	Real max = offset + gain*(angleOffset + angleRange)*REAL_2_PI/360.0;
	Real min = offset + gain*angleOffset*REAL_2_PI/360.0;
	if (max < min)
		std::swap(max, min);
	//printf("min = %f, max = %f\n", min, max);

	int encMin = posToEnc(min, angleOffset*REAL_2_PI/360.0, encoderOffset, encodersPerCycle, rotationDirection, offset, gain);
	Real angMin = posFromEnc(encMin, angleOffset*REAL_2_PI/360.0, encoderOffset, encodersPerCycle, rotationDirection, offset, gain);
	int encMax = posToEnc(max, angleOffset*REAL_2_PI/360.0, encoderOffset, encodersPerCycle, rotationDirection, offset, gain);
	Real angMax = posFromEnc(encMax, angleOffset*REAL_2_PI/360.0, encoderOffset, encodersPerCycle, rotationDirection, offset, gain);
	
	printf("angle (min, max) = (%f, %f)\n", angMin, angMax);
	printf("encoder (min, max) = (%i, %i)\n", encMin, encMax);
}

int main() {
	printf("Katana 300, KNI 3.2\n******************************************************************************************************************************\n");
	// joint #1
	test(1, 0.0, 348.0, 31000, 51200, +1, 30000, -REAL_PI, 1.0);
	// joint #2
	test(2, 124.25, -137.85, -31000, 94976, +1, -30000, 0.0, -1.0);
	// joint #3
	test(3, 52.7, 250.0, -31000, 81408, -1, -30000, REAL_PI, -1.0);
	// joint #4
	test(4, 63.5, 232.7, 31000, 51200, +1, 30000, -REAL_PI, +1.0);
	// joint #5
	test(5, 8.5, 342.8, 31000, 51200, +1, 30000, REAL_PI/*0.271071*/, -1.0); //

	printf("\nKatana 300, KNI 3.9.2\n******************************************************************************************************************************\n");
	// joint #1
	test(1, 6.65, 346.7, 31000, 51200, +1, 30000, -REAL_PI, 1.0);//new
	// joint #2
	test(2, 124.25, -143.0, -31000, 94976, +1, -30000, 0.0, -1.0);//new
	// joint #3
	test(3, 52.7, 127.3/*250.0*/, -31000, 81408, -1, -30000, REAL_PI, -1.0);
	// joint #4
	test(4, 63.5, 232.7, 31000, 51200, +1, 30000, -REAL_PI, +1.0);
	// joint #5
	test(5, 8.5, 342.8, 31000, 51200, +1, 30000, REAL_PI, -1.0);

	printf("\nKatana 450, KNI 4.x.x\n******************************************************************************************************************************\n");
	// joint #1
	test(1, 6.65, 339.0, 31000, 51200, +1, 30000, -REAL_PI, 1.0);//new
	// joint #2
	test(2, 124.25, -132.0, -31000, 94976, +1, -30000, 0.0, -1.0);//new
	// joint #3
	test(3, 52.7, 245.0, -31000, 47488, -1, -30000, REAL_PI, -1.0);
	// joint #4
	test(4, 63.5, 224.0, 31000, 51200, +1, 30000, -REAL_PI, +1.0);
	// joint #5
	test(5, 8.5, 336.0, 31000, 51200, +1, 30000, REAL_PI, -1.0);

	printf("\nKatana 450, AxNI\n******************************************************************************************************************************\n");
	// joint #1
	test(1, 6.65, 339.0, 100000/*31000*/, 51200, +1, 99000/*30000*/, -REAL_PI, 1.0);//new
	// joint #2
	test(2, 124.25, -132.0, 100000/*-31000*/, 94976, +1, 101000/*-30000*/, 0.0, -1.0);//new
	// joint #3
	test(3, 52.7, 127.4/*245.0*/, 100000/*-31000*/, 47488, -1, 101000/*-30000*/, REAL_PI, -1.0);
	// joint #4
	test(4, 63.5, 224.0, 100000/*31000*/, 51200, +1, 99000/*30000*/, -REAL_PI, +1.0);
	// joint #5
	test(5, 8.5, 336.0, 100000/*31000*/, 51200, +1, 99000/*30000*/, REAL_PI, -1.0);

	return 0;
}

