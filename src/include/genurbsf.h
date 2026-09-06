#ifndef NV_GENURBSF_H
#define NV_GENURBSF_H

#include "gegbl.h"
#include "gesurf.h"
#include "gept3dar.h"
#include "gedblar.h"
#pragma pack (push, 8)

class NvGeKnotVector;

class
GX_DLLEXPIMPORT
NvGeNurbSurface : public NvGeSurface
{
public:
   NvGeNurbSurface();
   NvGeNurbSurface(int degreeU, int degreeV, int propsInU, int propsInV,
                   int numControlPointsInU, int numControlPointsInV,
                   const NvGePoint3d controlPoints[],
                   const double weights[],
                   const NvGeKnotVector& uKnots,
                   const NvGeKnotVector& vKnots,
                   const NvGeTol& tol = NvGeContext::gTol);
   NvGeNurbSurface(const NvGeNurbSurface& nurb);

   // Assignment.
   //
   NvGeNurbSurface& operator = (const NvGeNurbSurface& nurb);

   // Geometric properties.
   //
   Nova::Boolean   isRationalInU      () const;
   Nova::Boolean   isPeriodicInU      (double&) const;
   Nova::Boolean   isRationalInV      () const;
   Nova::Boolean   isPeriodicInV      (double&) const;

   int singularityInU () const;
   int singularityInV () const;

   // Definition data.
   //
   int            degreeInU            () const;
   int            numControlPointsInU  () const;
   int            degreeInV            () const;
   int            numControlPointsInV  () const;
   void           getControlPoints     (NvGePoint3dArray& points) const;
   Nova::Boolean getWeights           (NvGeDoubleArray& weights) const;

   int       numKnotsInU    () const;
   void      getUKnots      (NvGeKnotVector& uKnots) const;

   int       numKnotsInV    () const;
   void      getVKnots      (NvGeKnotVector& vKnots) const;

   void      getDefinition  (int& degreeU, int& degreeV,
                             int& propsInU,	int& propsInV,
                             int& numControlPointsInU,
                             int& numControlPointsInV,
                             NvGePoint3dArray& controlPoints,
                             NvGeDoubleArray& weights,
                             NvGeKnotVector& uKnots,
                             NvGeKnotVector& vKnots) const;

   // Reset surface
   //
   NvGeNurbSurface& set     (int degreeU, int degreeV,
                             int propsInU, int propsInV,
                             int numControlPointsInU,
                             int numControlPointsInV,
                             const NvGePoint3d controlPoints[],
                             const double weights[],
                             const NvGeKnotVector& uKnots,
                             const NvGeKnotVector& vKnots,
                             const NvGeTol& tol = NvGeContext::gTol);

};

#pragma pack (pop)
#endif
