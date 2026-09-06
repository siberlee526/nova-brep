#include <NvException.h>
#include <gedblar.h>
#include <geintrvl.h>
#include <gekvec.h>

#include <gtest/gtest.h>

#include <initializer_list>

namespace
{
  // Knot tolerance used by every knot vector of this test.
  constexpr double THE_TEST_TOL = 1e-9;

  // Builds a knot vector from a list of knot values with the test tolerance.
  NvGeKnotVector MakeKnotVector (std::initializer_list<double> theKnots,
                                 double theEps = THE_TEST_TOL)
  {
    NvGeDoubleArray anArray;
    for (const double aKnot : theKnots)
    {
      anArray.append (aKnot);
    }
    return NvGeKnotVector (anArray, theEps);
  }

  // Checks the knot values of the whole vector against the expected list.
  void ExpectKnots (const NvGeKnotVector& theVector, std::initializer_list<double> theExpected)
  {
    ASSERT_EQ (theVector.length(), static_cast<int> (theExpected.size()));
    int anIndex = 0;
    for (const double aKnot : theExpected)
    {
      EXPECT_NEAR (theVector[anIndex], aKnot, THE_TEST_TOL) << "knot " << anIndex;
      ++anIndex;
    }
  }
}

// Pins the global knot tolerance: API users may change it, so the tests
// must not depend on whatever a previous test left behind.
class NvGeKnotVectorTest : public testing::Test
{
protected:
  void SetUp() override
  {
    NvGeKnotVector::globalKnotTolerance = THE_TEST_TOL;
  }
};

TEST_F (NvGeKnotVectorTest, DefaultConstructor_IsEmpty)
{
  const NvGeKnotVector aVector;
  EXPECT_TRUE (aVector.isEmpty());
  EXPECT_EQ (aVector.length(), 0);
  EXPECT_EQ (aVector.logicalLength(), 0);
}

TEST_F (NvGeKnotVectorTest, SizeGrowConstructor_ZeroesKnotsAndSetsGrowLength)
{
  NvGeKnotVector aVector (3, 5);
  EXPECT_EQ (aVector.length(), 3);
  EXPECT_EQ (aVector.growLength(), 5);
  for (int anIndex = 0; anIndex < 3; ++anIndex)
  {
    EXPECT_NEAR (aVector[anIndex], 0.0, THE_TEST_TOL);
  }

  EXPECT_THROW (NvGeKnotVector (-1, 5), NvException);
  EXPECT_THROW (NvGeKnotVector (3, 0), NvException);
}

TEST_F (NvGeKnotVectorTest, ArrayConstructor_CopiesKnotsVerbatim)
{
  const double aKnots[] = {0.0, 0.0, 1.0, 2.0, 2.0, 2.0};
  const NvGeKnotVector aVector (6, aKnots);

  EXPECT_EQ (aVector.length(), 6);
  ExpectKnots (aVector, {0.0, 0.0, 1.0, 2.0, 2.0, 2.0});
  EXPECT_NEAR (aVector.startParam(), 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aVector.endParam(), 2.0, THE_TEST_TOL);

  EXPECT_THROW (NvGeKnotVector (-1, aKnots), NvException);
  EXPECT_THROW (NvGeKnotVector (2, static_cast<const double*> (nullptr)), NvException);
}

TEST_F (NvGeKnotVectorTest, PlusMultConstructor_ElevatesDistinctKnotMultiplicities)
{
  const NvGeKnotVector aSrc = MakeKnotVector ({0.0, 0.0, 1.0, 2.0, 2.0, 2.0});

  const NvGeKnotVector aRaised (2, aSrc);
  EXPECT_EQ (aRaised.length(), 12);
  // {0 x2, 1 x1, 2 x3} elevated by 2 becomes {0 x4, 1 x3, 2 x5}.
  ExpectKnots (aRaised, {0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 2.0, 2.0, 2.0, 2.0, 2.0});

  EXPECT_THROW (NvGeKnotVector (-1, aSrc), NvException);
}

TEST_F (NvGeKnotVectorTest, DoubleArrayConstructor_CopiesArrayAndTolerance)
{
  NvGeDoubleArray anArray;
  anArray.append (0.0);
  anArray.append (1.0);
  anArray.append (2.0);

  const NvGeKnotVector aVector (anArray, 1e-7);
  EXPECT_EQ (aVector.length(), 3);
  ExpectKnots (aVector, {0.0, 1.0, 2.0});
  EXPECT_NEAR (aVector.tolerance(), 1e-7, 1e-15);
}

TEST_F (NvGeKnotVectorTest, CopyConstructor_DeepCopiesBuffer)
{
  NvGeKnotVector aSrc = MakeKnotVector ({0.0, 1.0, 2.0});
  aSrc.setGrowLength (4);

  const NvGeKnotVector aCopy (aSrc);
  EXPECT_EQ (aCopy.length(), 3);
  EXPECT_EQ (aCopy.growLength(), 4);
  EXPECT_NEAR (aCopy.tolerance(), THE_TEST_TOL, 1e-15);

  aSrc[0] = 5.0;
  aSrc.setTolerance (1e-3);
  EXPECT_NEAR (aCopy[0], 0.0, THE_TEST_TOL); // deep copy: unaffected by the source
  EXPECT_NEAR (aCopy.tolerance(), THE_TEST_TOL, 1e-15);
}

TEST_F (NvGeKnotVectorTest, AssignmentOperator_DeepCopiesAndKeepsSelfAssignmentSafe)
{
  NvGeKnotVector aVec = MakeKnotVector ({0.0, 1.0, 2.0});
  NvGeKnotVector anOther = MakeKnotVector ({5.0});

  aVec = anOther;
  ExpectKnots (aVec, {5.0});

  anOther[0] = 9.0;
  EXPECT_NEAR (aVec[0], 5.0, THE_TEST_TOL); // deep copy

  aVec.setTolerance (1e-5);
  NvGeKnotVector& aRef = aVec;
  aVec = aRef; // self assignment must be a harmless no-op
  ExpectKnots (aVec, {5.0});
  EXPECT_NEAR (aVec.tolerance(), 1e-5, 1e-15);
}

TEST_F (NvGeKnotVectorTest, AssignmentFromDoubleArray_ReplacesKnotsKeepsTolerance)
{
  NvGeKnotVector aVec = MakeKnotVector ({0.0, 1.0});

  NvGeDoubleArray anArray;
  anArray.append (3.0);
  anArray.append (4.0);
  anArray.append (5.0);
  aVec = anArray;

  EXPECT_EQ (aVec.length(), 3);
  ExpectKnots (aVec, {3.0, 4.0, 5.0});
  EXPECT_NEAR (aVec.tolerance(), THE_TEST_TOL, 1e-15);
}

TEST_F (NvGeKnotVectorTest, Append_MaintainsOrderAndReturnsIndexOfNewKnot)
{
  NvGeKnotVector aVec;
  EXPECT_EQ (aVec.append (0.0), 0);
  EXPECT_EQ (aVec.append (1.0), 1);
  EXPECT_EQ (aVec.append (1.0), 2); // a knot equal to the last one is allowed

  EXPECT_THROW (aVec.append (0.5), NvException); // below the last knot
  EXPECT_EQ (aVec.length(), 3);

  EXPECT_EQ (aVec.append (1.0 - 1e-12), 3); // within knot tolerance of the last knot
  EXPECT_EQ (aVec.length(), 4);
}

TEST_F (NvGeKnotVectorTest, AppendVector_ConcatenatesTailWithJunctionRemapping)
{
  // The tail is shifted so that its first knot coincides with the current
  // end parameter; the junction knot appears twice (C0 junction).
  NvGeKnotVector aHead = MakeKnotVector ({0.0, 1.0, 2.0});
  NvGeKnotVector aTail = MakeKnotVector ({5.0, 6.0, 7.0});

  aHead.append (aTail);
  ExpectKnots (aHead, {0.0, 1.0, 2.0, 2.0, 3.0, 4.0});

  NvGeKnotVector anEmpty;
  NvGeKnotVector anOrphan = MakeKnotVector ({1.0, 2.0});
  anEmpty.append (anOrphan);
  ExpectKnots (anEmpty, {1.0, 2.0}); // joining to an empty vector adopts the tail

  EXPECT_THROW (aHead.append (aTail, 2.0), NvException);  // ratio above 1
  EXPECT_THROW (aHead.append (aTail, -0.1), NvException); // ratio below 0
}

TEST_F (NvGeKnotVectorTest, AppendVector_CommonKnotsAtJunctionAreShared)
{
  // Half of the tail knots (two) are declared common with the closing knots
  // of the head; they must coincide and are not appended again.
  NvGeKnotVector aHead = MakeKnotVector ({0.0, 1.0, 2.0, 3.0});
  NvGeKnotVector aTail = MakeKnotVector ({2.0, 3.0, 4.0, 5.0});

  aHead.append (aTail, 0.5);
  ExpectKnots (aHead, {0.0, 1.0, 2.0, 3.0, 4.0, 5.0});

  // Declaring knots common that do not coincide with the head end must
  // fail: the shifted tail {1.0, 1.5} cannot share two knots with the
  // head {0, 1, 2} (0.5 * 4 knots = 2 common knots, exactly).
  NvGeKnotVector aMismatch = MakeKnotVector ({0.0, 1.0, 2.0});
  NvGeKnotVector aBadTail = MakeKnotVector ({2.0, 2.5, 3.0, 4.0});
  EXPECT_THROW (aMismatch.append (aBadTail, 0.5), NvException);
}

TEST_F (NvGeKnotVectorTest, InsertAt_PlacesKnotsWithMultiplicity)
{
  NvGeKnotVector aVec = MakeKnotVector ({0.0, 1.0, 2.0});

  aVec.insertAt (1, 0.5, 2);
  ExpectKnots (aVec, {0.0, 0.5, 0.5, 1.0, 2.0});

  aVec.insertAt (aVec.length(), 3.0); // at the end is a valid position
  ExpectKnots (aVec, {0.0, 0.5, 0.5, 1.0, 2.0, 3.0});
}

TEST_F (NvGeKnotVectorTest, InsertAt_ViolatesOrderOrRange_Throws)
{
  NvGeKnotVector aVec = MakeKnotVector ({0.0, 1.0, 2.0});

  EXPECT_THROW (aVec.insertAt (1, 5.0), NvException);   // above the next knot
  EXPECT_THROW (aVec.insertAt (1, -1.0), NvException);  // below the previous knot
  EXPECT_THROW (aVec.insertAt (4, 1.0), NvException);   // index beyond length
  EXPECT_THROW (aVec.insertAt (-1, 1.0), NvException);  // negative index
  EXPECT_THROW (aVec.insertAt (0, 1.0, 0), NvException); // multiplicity below one
}

TEST_F (NvGeKnotVectorTest, Insert_KeepsSortedPosition)
{
  NvGeKnotVector aVec = MakeKnotVector ({0.0, 1.0, 2.0});

  aVec.insert (1.5);
  ExpectKnots (aVec, {0.0, 1.0, 1.5, 2.0});

  aVec.insert (5.0);
  ExpectKnots (aVec, {0.0, 1.0, 1.5, 2.0, 5.0});

  aVec.insert (-1.0);
  ExpectKnots (aVec, {-1.0, 0.0, 1.0, 1.5, 2.0, 5.0});

  aVec.insert (0.0); // duplicates after the existing equal knots
  ExpectKnots (aVec, {-1.0, 0.0, 0.0, 1.0, 1.5, 2.0, 5.0});
}

TEST_F (NvGeKnotVectorTest, RemoveAtAndRemoveSubVector_EditInRange)
{
  NvGeKnotVector aVec = MakeKnotVector ({0.0, 1.0, 2.0, 3.0, 4.0});

  aVec.removeAt (1);
  ExpectKnots (aVec, {0.0, 2.0, 3.0, 4.0});

  aVec.removeSubVector (1, 2);
  ExpectKnots (aVec, {0.0, 4.0});
}

TEST_F (NvGeKnotVectorTest, RemoveAtAndRemoveSubVector_IndexErrorsThrow)
{
  NvGeKnotVector aVec = MakeKnotVector ({0.0, 1.0, 2.0});

  EXPECT_THROW (aVec.removeAt (3), NvException);
  EXPECT_THROW (aVec.removeAt (-1), NvException);
  EXPECT_THROW (aVec.removeSubVector (0, 3), NvException);
  EXPECT_THROW (aVec.removeSubVector (1, 0), NvException);
}

TEST_F (NvGeKnotVectorTest, Reverse_ReversesSequenceAndRemapsValues)
{
  // ARX semantics: knot v becomes (a + b) - v with a = 0 and b = 5, so the
  // reversed sequence stays non-decreasing:
  // {0, 1, 2, 5} -> {5-5, 5-2, 5-1, 5-0} = {0, 3, 4, 5}.
  NvGeKnotVector aVec = MakeKnotVector ({0.0, 1.0, 2.0, 5.0});
  aVec.reverse();
  ExpectKnots (aVec, {0.0, 3.0, 4.0, 5.0});
  EXPECT_NEAR (aVec.startParam(), 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aVec.endParam(), 5.0, THE_TEST_TOL);

  NvGeKnotVector anEmpty;
  anEmpty.reverse(); // no-op, must not throw
  EXPECT_TRUE (anEmpty.isEmpty());
}

TEST_F (NvGeKnotVectorTest, Split_SplitsAtIntermediateParameter)
{
  const NvGeKnotVector aVec = MakeKnotVector ({0.0, 1.0, 2.0, 3.0, 4.0, 5.0});

  NvGeKnotVector aLeft;
  NvGeKnotVector aRight;
  const int aSplitIndex = aVec.split (2.5, &aLeft, 3, &aRight, 4);

  EXPECT_EQ (aSplitIndex, 3); // knots 0, 1, 2 went into the left part
  ExpectKnots (aLeft,  {0.0, 1.0, 2.0, 2.0, 2.0});      // trailing 2 in total multLast = 3 copies
  ExpectKnots (aRight, {2.5, 2.5, 2.5, 2.5, 3.0, 4.0, 5.0}); // leading 2.5 in total multFirst = 4 copies
}

TEST_F (NvGeKnotVectorTest, Split_ParameterOnExistingKnot_TrimsOrExtendsMultiplicity)
{
  const NvGeKnotVector aVec = MakeKnotVector ({0.0, 1.0, 2.0, 3.0});

  NvGeKnotVector aLeft;
  NvGeKnotVector aRight;
  const int aSplitIndex = aVec.split (2.0, &aLeft, 1, &aRight, 2);

  EXPECT_EQ (aSplitIndex, 3);
  ExpectKnots (aLeft,  {0.0, 1.0, 2.0});     // multiplicity already 1, unchanged
  ExpectKnots (aRight, {2.0, 2.0, 3.0});     // opening knot duplicated
}

TEST_F (NvGeKnotVectorTest, Split_NullOutputsReturnSplitIndexOnly)
{
  const NvGeKnotVector aVec = MakeKnotVector ({0.0, 1.0, 2.0, 3.0});
  EXPECT_EQ (aVec.split (2.5, nullptr, 2, nullptr, 2), 3);
}

TEST_F (NvGeKnotVectorTest, Split_InvalidParameterOrMultiplicity_Throws)
{
  const NvGeKnotVector aVec = MakeKnotVector ({0.0, 1.0, 2.0, 3.0});

  NvGeKnotVector aLeft;
  NvGeKnotVector aRight;
  EXPECT_THROW (aVec.split (10.0, &aLeft, 1, &aRight, 1), NvException);
  EXPECT_THROW (aVec.split (0.5, &aLeft, 0, &aRight, 1), NvException);
  EXPECT_THROW (NvGeKnotVector().split (0.0, &aLeft, 1, &aRight, 1), NvException);
}

TEST_F (NvGeKnotVectorTest, GetInterval_ReturnsSpanContainingParameter)
{
  const NvGeKnotVector aVec = MakeKnotVector ({0.0, 1.0, 2.0, 3.0, 4.0, 5.0});

  NvGeInterval anInterval;
  EXPECT_EQ (aVec.getInterval (2, 1.5, anInterval), 1);
  EXPECT_NEAR (anInterval.lowerBound(), 1.0, THE_TEST_TOL);
  EXPECT_NEAR (anInterval.upperBound(), 2.0, THE_TEST_TOL);

  EXPECT_EQ (aVec.getInterval (2, 2.5, anInterval), 2);
  EXPECT_NEAR (anInterval.lowerBound(), 2.0, THE_TEST_TOL);
  EXPECT_NEAR (anInterval.upperBound(), 3.0, THE_TEST_TOL);

  // The domain end belongs to the last span of the order-2 curve.
  EXPECT_EQ (aVec.getInterval (2, 4.0, anInterval), 3);
  EXPECT_NEAR (anInterval.lowerBound(), 3.0, THE_TEST_TOL);
  EXPECT_NEAR (anInterval.upperBound(), 4.0, THE_TEST_TOL);
}

TEST_F (NvGeKnotVectorTest, GetInterval_ParameterOutsideDomainOrInvalidOrder_Throws)
{
  const NvGeKnotVector aVec = MakeKnotVector ({0.0, 1.0, 2.0, 3.0, 4.0, 5.0});

  NvGeInterval anInterval;
  // The order-2 domain is [knot[1], knot[4]] = [1, 4].
  EXPECT_THROW (aVec.getInterval (2, 0.5, anInterval), NvException);
  EXPECT_THROW (aVec.getInterval (2, 4.5, anInterval), NvException);
  EXPECT_THROW (aVec.getInterval (0, 1.5, anInterval), NvException);
}

TEST_F (NvGeKnotVectorTest, GetDistinctKnots_CollapsesRunsOfEqualKnots)
{
  const NvGeKnotVector aVec = MakeKnotVector ({0.0, 0.0, 1.0, 2.0, 2.0, 2.0});

  NvGeDoubleArray aDistinct;
  aVec.getDistinctKnots (aDistinct);

  ASSERT_EQ (aDistinct.length(), 3);
  EXPECT_NEAR (aDistinct[0], 0.0, THE_TEST_TOL);
  EXPECT_NEAR (aDistinct[1], 1.0, THE_TEST_TOL);
  EXPECT_NEAR (aDistinct[2], 2.0, THE_TEST_TOL);

  // The output array is replaced, not appended to.
  aVec.getDistinctKnots (aDistinct);
  EXPECT_EQ (aDistinct.length(), 3);
}

TEST_F (NvGeKnotVectorTest, Contains_ChecksRangeMembership)
{
  const NvGeKnotVector aVec = MakeKnotVector ({0.0, 1.0, 2.0});

  EXPECT_TRUE  (aVec.contains (0.0));
  EXPECT_TRUE  (aVec.contains (1.5));
  EXPECT_TRUE  (aVec.contains (2.0));
  EXPECT_FALSE (aVec.contains (2.5));
  EXPECT_FALSE (aVec.contains (-0.5));

  const NvGeKnotVector anEmpty;
  EXPECT_FALSE (anEmpty.contains (0.0));
}

TEST_F (NvGeKnotVectorTest, IsOn_ChecksKnotMembership)
{
  const NvGeKnotVector aVec = MakeKnotVector ({0.0, 1.0, 2.0});

  EXPECT_TRUE  (aVec.isOn (1.0));
  EXPECT_TRUE  (aVec.isOn (1.0 + 1e-12)); // within the knot tolerance
  EXPECT_FALSE (aVec.isOn (1.5));
  EXPECT_FALSE (aVec.isOn (5.0));

  const NvGeKnotVector anEmpty;
  EXPECT_FALSE (anEmpty.isOn (0.0));
}

TEST_F (NvGeKnotVectorTest, MultiplicityAt_ReportsFullRunsOfEqualKnots)
{
  const NvGeKnotVector aVec = MakeKnotVector ({0.0, 0.0, 1.0, 2.0, 2.0, 2.0});

  EXPECT_EQ (aVec.multiplicityAt (0), 2);
  EXPECT_EQ (aVec.multiplicityAt (2), 1); // the single 1.0 knot sits at index 2
  EXPECT_EQ (aVec.multiplicityAt (5), 3);  // middle of the trailing run

  EXPECT_THROW (aVec.multiplicityAt (6), NvException);
  EXPECT_THROW (aVec.multiplicityAt (-1), NvException);
}

TEST_F (NvGeKnotVectorTest, NumIntervals_CountsDistinctSpans)
{
  const NvGeKnotVector aVec = MakeKnotVector ({0.0, 0.0, 1.0, 2.0, 2.0, 2.0});
  EXPECT_EQ (aVec.numIntervals(), 2); // distinct knots {0, 1, 2} span 2 intervals

  EXPECT_EQ (MakeKnotVector ({0.0, 0.0, 0.0}).numIntervals(), 0);
  EXPECT_EQ (NvGeKnotVector().numIntervals(), 0);
}

TEST_F (NvGeKnotVectorTest, SetRange_RescalesKnotsLinearly)
{
  NvGeKnotVector aVec = MakeKnotVector ({0.0, 1.0, 2.0, 3.0});
  aVec.setRange (10.0, 20.0);

  EXPECT_NEAR (aVec[0], 10.0, THE_TEST_TOL);
  EXPECT_NEAR (aVec[1], 10.0 + 10.0 / 3.0, THE_TEST_TOL);
  EXPECT_NEAR (aVec[2], 10.0 + 20.0 / 3.0, THE_TEST_TOL);
  EXPECT_NEAR (aVec[3], 20.0, THE_TEST_TOL);

  // A degenerate source range collapses onto the new lower bound.
  NvGeKnotVector aFlat = MakeKnotVector ({2.0, 2.0, 2.0});
  aFlat.setRange (5.0, 9.0);
  ExpectKnots (aFlat, {5.0, 5.0, 5.0});

  EXPECT_THROW (aVec.setRange (5.0, 1.0), NvException);
}

TEST_F (NvGeKnotVectorTest, ToleranceAccessors_RoundTrip)
{
  NvGeKnotVector aVec (1e-6);
  EXPECT_NEAR (aVec.tolerance(), 1e-6, 1e-15);

  aVec.setTolerance (1e-12);
  EXPECT_NEAR (aVec.tolerance(), 1e-12, 1e-15);
}

TEST_F (NvGeKnotVectorTest, LengthAndCapacityAccessors_ResizeAndValidate)
{
  NvGeKnotVector aVec = MakeKnotVector ({1.0, 2.0});

  aVec.setLogicalLength (4);
  EXPECT_EQ (aVec.logicalLength(), 4);
  EXPECT_NEAR (aVec[2], 0.0, THE_TEST_TOL); // newly exposed knots are zeroed
  EXPECT_NEAR (aVec[3], 0.0, THE_TEST_TOL);

  aVec.setLogicalLength (1);
  EXPECT_EQ (aVec.logicalLength(), 1);

  aVec.setPhysicalLength (10);
  EXPECT_EQ (aVec.physicalLength(), 10);

  aVec.setGrowLength (7);
  EXPECT_EQ (aVec.growLength(), 7);

  EXPECT_THROW (aVec.setLogicalLength (-1), NvException);
  EXPECT_THROW (aVec.setPhysicalLength (-1), NvException);
  EXPECT_THROW (aVec.setGrowLength (0), NvException);
}

TEST_F (NvGeKnotVectorTest, Set_ReplacesKnotsAndTolerance)
{
  NvGeKnotVector aVec = MakeKnotVector ({9.0});
  const double aKnots[] = {0.0, 1.0, 2.0};

  aVec.set (3, aKnots, 1e-7);
  EXPECT_EQ (aVec.length(), 3);
  ExpectKnots (aVec, {0.0, 1.0, 2.0});
  EXPECT_NEAR (aVec.tolerance(), 1e-7, 1e-15);

  EXPECT_THROW (aVec.set (-1, aKnots), NvException);
  EXPECT_THROW (aVec.set (1, static_cast<const double*> (nullptr)), NvException);
}

TEST_F (NvGeKnotVectorTest, IsEqualTo_ComparesComponentWiseWithinTolerance)
{
  const NvGeKnotVector aVec = MakeKnotVector ({0.0, 1.0, 2.0});
  const NvGeKnotVector aClose = MakeKnotVector ({0.0, 1.0 + 5e-10, 2.0});
  const NvGeKnotVector aFar = MakeKnotVector ({0.0, 1.0 + 1e-6, 2.0});
  const NvGeKnotVector aShorter = MakeKnotVector ({0.0, 1.0});

  EXPECT_TRUE  (aVec.isEqualTo (aClose));  // 5e-10 is within the 1e-9 tolerance
  EXPECT_FALSE (aVec.isEqualTo (aFar));
  EXPECT_FALSE (aVec.isEqualTo (aShorter)); // different lengths
}

TEST_F (NvGeKnotVectorTest, StartAndEndParam_EmptyVector_Throws)
{
  const NvGeKnotVector anEmpty;
  EXPECT_THROW (anEmpty.startParam(), NvException);
  EXPECT_THROW (anEmpty.endParam(), NvException);
}

TEST_F (NvGeKnotVectorTest, AsArrayPtr_ExposesRawStorage)
{
  NvGeKnotVector aVec = MakeKnotVector ({0.0, 1.0, 2.0});

  double* aRaw = aVec.asArrayPtr();
  aRaw[1] = 1.5;
  EXPECT_NEAR (aVec[1], 1.5, THE_TEST_TOL);

  const NvGeKnotVector& aConstRef = aVec;
  EXPECT_NEAR (aConstRef.asArrayPtr()[2], 2.0, THE_TEST_TOL);
}
