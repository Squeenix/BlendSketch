/*
* ***** BEGIN GPL LICENSE BLOCK *****
*
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License
* as published by the Free Software Foundation; either version 2
* of the License, or (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software Foundation,
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
*
* The Original Code is Copyright (C) 2016 Blender Foundation.
* All rights reserved.
*
*
* Contributor(s): Ryan Annis
*
*
* ***** END GPL LICENSE BLOCK *****
*/

/** \file blender/editors/mesh/editmesh_sketch.c
*  \ingroup edmesh
*
*  Sketch based tool that forces the shape of the sihoutte to conform with the drawn sketch
*/

#include "WM_api.h"
#include "WM_types.h"

#include "DNA_object_types.h"

#include "UI_interface.h"
#include "UI_resources.h"

#include "DNA_screen_types.h"

#include "BIF_gl.h"

#include "BKE_context.h"

#include "DNA_view3d_types.h"
#include "ED_view3d.h"

typedef FloatPairLinked;

/* Used when we need some linked list/stack implmentation.*/
typedef struct FloatPairLinked {
	float a;
	float b;
	FloatPairLinked* next;
} SketchPoint;

/* Used to represent point on the sihouette before it has been simplified (where
 * subpixel precision is necessary).*/
typedef struct IntPair {
	int x;
	int y;
} SilhouettePixel;

/* Used to represent line being drawn on screen by user and a point on the sihouette
 * after it has been simplified and converted to a polyline.*/
typedef struct FloatPair {
	float x;
	float y;
} SketchPoint;

/* Stores the extracted sillhouete of screen space as a polyline.
 * Each member of indices indices_n stores the number of points in
 * the edge edge_n.*/
typedef struct Sihouette {
	SketchPoint *pointBuffer;
	int *indicesBuffer;
	int numberOfIndices;
	int numberOfPoints;
} Silhouette;

static const int INITAL_POINT_BUFFER_SIZE = 128;
static const float STROKE_LINE_WIDTH_PX = 10;

/* Number of points must be >= 2*/
typedef struct SketchStroke {
	int curSize;
	int maxSize;
	SketchPoint *pointData;
} SketchStroke;

typedef struct SketchMorphTool_OpData {
	ARegion *activatedRegion; /*Region that the sketch editor tool was activated in.*/
	SketchStroke *pointList; /* Structure for storing the list of points.*/
	ViewDepths *depthBuffer;
	Silhouette *Silhouette;
} SketchMorphTool_OpData;

/* Needed data structures for curve matching algorithm*/
/* Line equations (in slope-equation form)
 * Horizon Trees (SlopeLineEq[]):  eg m1 m2 means that m1 intersects m2
 and that m1 is on the left side of m2 (we don't order by slope here)
 a list of integers i which hold which integers intersect on the right
 endpoint (we can use this to determine the size of horizon trees)
 M: edges delimiting the left side of the cut
 N: edges delimiting the right side of the cut*/

typedef struct SlopeLineEq {
	int m;
	int b;
} SlopeLineEq;


/*******************************************************
* SECTION ON GENERAL AND UI SETUP RELATED FUNCTIONALITY*
********************************************************/

enum {
	PLACEHOLDER = 0
};

wmKeyMap *sketchtool_modal_keymap(wmKeyConfig *keyconf){
	static EnumPropertyItem modal_items[] = {
		{ PLACEHOLDER, "CANCEL", 0, "Cancel", "" },
	};

	wmKeyMap *keymap = WM_modalkeymap_get(keyconf, "Sketch Morph Tool Modal Keymap");

	/* Return if keymap is already set */
	if (keymap && keymap->modal_items)
		return NULL;

	keymap = WM_modalkeymap_add(keyconf, "Sketch Morph Tool Modal Keymap", modal_items);

	/* items for modal map */

	WM_modalkeymap_assign(keymap, "MESH_OT_sketch_morph_tool");

	return keymap;
}

static void sketchMorphTool_init(const bContext *C, SketchMorphTool_OpData *data){
	//Scene *scene = CTX_data_scene(C);
	Object *obedit = CTX_data_edit_object(C);
	//Region *region = CTX_wm_region(C);
}

/* Generates and assigns the OpenGL VBOs for drawing the strokes.
 * Each line segment takes two additional tris.  The vertexList
 * forming the edges of the tris is offset (stroke width)/2
 * along the normal of the lines.*/
static void generateStrokeVBOs(const SketchMorphTool_OpData *data, GLuint *vertices, GLuint *indices){
	SketchStroke *points = data->pointList;
	SketchPoint *pointBuffer = points->pointData;

	int numberOfPoints = points->curSize;

	/* Don't draw the stroke if it has less than two points..*/
	if (numberOfPoints < 2){
		return;
	}

	/* Arrays to be passed to GPU.*/
	int vertexListSize = sizeof(float) * 4 * numberOfPoints;
	float *vertexList = malloc(vertexListSize);

	int indexListSize = sizeof(unsigned int) * 3 * 2 * (numberOfPoints - 1);
	int *indexList = malloc(indexListSize);

	/* First, calculate the derivative of each line segment.
	 * We treat the derivative of each vertex to be the averaged
	 * derative of it's two neighbouring line segments.
	 * This is done in bulk because it saves square roots
	 * to first convert the derivatives of each line segment
	 * to unit vectors. */

	SketchPoint* lineSegmentNormals = malloc(sizeof(SketchPoint) * numberOfPoints - 1);

	for (int i = 0; i < numberOfPoints - 1; i++){
		float dX = pointBuffer[i + 1].x - pointBuffer[i].x;
		float dY = pointBuffer[i + 1].x - pointBuffer[i].y;
		float magnitude = sqrt(dX * dX + dY * dY);
		lineSegmentNormals[i].x = dX / magnitude;
		lineSegmentNormals[i].y = dY / magnitude;
	}

	/* Calculate the location of each vertex.
	 * We do the first and last points first to avoid branching inside the for loop */
	vertexList[0] = pointBuffer[0].x + STROKE_LINE_WIDTH_PX * lineSegmentNormals[0].x;
	vertexList[1] = pointBuffer[0].y + STROKE_LINE_WIDTH_PX * lineSegmentNormals[0].y;
	vertexList[2] = pointBuffer[0].x - STROKE_LINE_WIDTH_PX * lineSegmentNormals[0].x;
	vertexList[3] = pointBuffer[0].y - STROKE_LINE_WIDTH_PX * lineSegmentNormals[0].y;

	vertexList[(numberOfPoints - 1) * 4] = pointBuffer[numberOfPoints - 1].x + STROKE_LINE_WIDTH_PX * lineSegmentNormals[numberOfPoints - 1].x;
	vertexList[(numberOfPoints - 1) * 4 + 1] = pointBuffer[numberOfPoints - 1].y + STROKE_LINE_WIDTH_PX * lineSegmentNormals[numberOfPoints - 1].y;
	vertexList[(numberOfPoints - 1) * 4 + 2] = pointBuffer[numberOfPoints - 1].x - STROKE_LINE_WIDTH_PX * lineSegmentNormals[numberOfPoints - 1].x;
	vertexList[(numberOfPoints - 1) * 4 + 3] = pointBuffer[numberOfPoints - 1].y - STROKE_LINE_WIDTH_PX * lineSegmentNormals[numberOfPoints - 1].y;

	for (int i = 1; i < numberOfPoints - 1; i++){
		vertexList[i * 4] = pointBuffer[i].x + STROKE_LINE_WIDTH_PX * lineSegmentNormals[i].x;
		vertexList[i * 4 + 1] = pointBuffer[i].y + STROKE_LINE_WIDTH_PX * lineSegmentNormals[i].y;
		vertexList[i * 4 + 2] = pointBuffer[i].x + STROKE_LINE_WIDTH_PX * lineSegmentNormals[i].x;
		vertexList[i * 4 + 3] = pointBuffer[i].y + STROKE_LINE_WIDTH_PX * lineSegmentNormals[i].y;
	}

	/* Calculate the indexList for each triangle CCW winding order.
	 * Indexes for the vertex array are laid out as such.
	 * [i]
	 * ||||||||||||||			[i+2]
	 * |||||||||||||||||||||||||
	 * |||||||||||||||||||||||||
	 * |||||||||||||||||||||||||
	 * ||||||||||||||			[i+3]
	 * [i + 1]
	 * */
	for (int i = 0; i < numberOfPoints - 1; i++){
		/* "Top half" triangle */
		indexList[i * 4] = i;
		indexList[i * 4 + 1] = i + 1;
		indexList[i * 4 + 2] = i + 2;

		/* "Bottom half' triangle */
		indexList[i * 4 + 3] = i + 1;
		indexList[i * 4 + 4] = i + 3;
		indexList[i * 4 + 5] = i + 2;
	}
	glGenBuffers(2, vertices, indices);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indices);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, indexListSize, &indexList[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, vertices);

	free(vertexList);
	free(indexList);

}

static void sketchToolDraw(const bContext *C, SketchMorphTool_OpData *data){
	View3D *v3d = CTX_wm_view3d(C);
	GLuint vertices;
	GLuint indices;
	generateStrokeVBOs(data, vertices, indices);
}

/* Adds a point to the current sketch line being drawn.*/
static void sketch_morph_add_point(SketchMorphTool_OpData *op, float x, float y){
	SketchPoint newPoint = { x, y };
	/* Initializes the buffer with space for 128 points
	 * and doubles the size if the buffer runs out of space.*/
	
	if (!(op->pointList)){
		/* Inits position data buffer on the heap.*/
		SketchStroke *posData = malloc(sizeof(SketchStroke));
		SketchPoint *pointBuffer = malloc(sizeof(SketchPoint) * INITAL_POINT_BUFFER_SIZE);
		SketchStroke tempPosData = { 0, INITAL_POINT_BUFFER_SIZE, pointBuffer };
		*posData = tempPosData;

		op->pointList = posData;
	}

	if (op->pointList->curSize == op->pointList->maxSize){
		SketchPoint *newBuffer = malloc(sizeof(SketchPoint) * op->pointList->maxSize * 2);
		memcpy(newBuffer, op->pointList->pointData, op->pointList->curSize * sizeof(SketchPoint));
		free(op->pointList->pointData);
	}

	/* Add the point*/
	op->pointList->pointData[op->pointList->curSize] = newPoint;
	op->pointList->curSize++;
}

/* Convenience function.  We are storing the points as floats because sub-pixel-presicion
 * accuracy is useful when doing the edge filter convolutions. */
static void sketch_morph_add_point_i(SketchMorphTool_OpData *op, int x, int y){
	sketch_morph_add_point(op, (float)x, (float)y);
}

/*Updates and retrieves the depth buffer of the region being acted upon.*/
static ViewDepths *retrieveDepthBuffer(const ARegion *ar){
	ED_view3d_depth_update(ar);
	RegionView3D *regionView = ar->regiondata;
	ViewDepths *depths = regionView->depths;
	return depths;
}

/* Allowing the user's stroke to overlap itself will lead to screwed up geometry.
 * Naive solution and runs in O(N^2).*/
static bool checkUserStrokeOverlaps(SketchMorphTool_OpData *op){
	SketchStroke *points = op->pointList;
	SketchPoint *pointBuffer = points->curSize;

	for (int i = 0; i < points->curSize - 2; i++){
		SketchPoint a1 = pointBuffer[i];
		SketchPoint a2 = pointBuffer[i + 1];

		for (int p = i + 1; p < points->curSize - 1; p++){
			SketchPoint b1 = pointBuffer[p];
			SketchPoint b2 = pointBuffer[p + 1];

			/* Check if set of two line segments overlap*/
			float denominator = ((a2.x - a1.x) * (b2.y - b1.y)) - ((a2.y - a1.y) * (b2.x - b1.x));
			float numerator1 = ((a1.y - b1.y) * (b2.x - b1.x)) - ((a1.x - b1.x) * (b2.y - b1.y));
			float numerator2 = ((a1.y - b1.y) * (a2.x - a1.x)) - ((a1.x - b1.x) * (a2.y - a1.y));

			// Detect coincident lines
			if (denominator == 0) return numerator1 == 0 && numerator2 == 0;

			float r = numerator1 / denominator;
			float s = numerator2 / denominator;

			return (r >= 0 && r <= 1) && (s >= 0 && s <= 1);
		}
	}

	return false;
}





/**************************************************************************
 * SECTION ON PROCESSING USER INPUT AND THE VIEWPORT SCREEN INTO POLYLINES*
 **************************************************************************/

/* Used in tracing pixel lines into polylines.  It is overlaid on the screen,
 * rotated to match the current direction of the polyline, and then used to
 * select the next pixel to add to the polyline (where lower values mean a
 * higher priority).
 */
static char priorityMask[8] = { 1, 0, 1, 2, 3, WCHAR_MAX, 3, 2 };

/* Used to efficiently map the range  [0,7] -> [-1, 1][-1, 1] =/ 0
* Alternates between x and y values. */
static char offsetMap[16] = { -1, -1, 0, -1, 1, -1, 1, 0, 1, 1, 0, 1, -1, 1, -1, 0 };



/* Use a convolution kernel to perform a simple laplacian edge detection filter on
 * The depth buffer.  Then applies a threshold function to filter out noise.
 * Then filters out sharp edges by erasing points with less than 2 depth
 * continuous neighbours twice.
 *
 * numActivePixels returns the number of pixels with a non-zero value and is used
 * to optimize the initial size of the pixel buffer when mapping pixels to polylines.
 *
 * The returned depth buffer has a range [0, 1]. */
static float* preprocessDepthMap(const ViewDepths *depths, float depthContinuityThresholdSquared, int *numActivePixels){
	const float kernel[3][3] = { { 0, -1, 0 },
	{ -1, 4, -1 },
	{ 0, -1, 0 } };
	const int kernelSize = 3;
	const int kernelCenter = 1;
	const int kernelOffset = kernelCenter - kernelCenter;

	numActivePixels = 0;

	/*After applying the filter, anything below this value will be discarded as noise.*/
	const float threshold = 0.005;

	/* We can convert from specified depth range to our range of
	 * [0, 1] using the formula: (depth - minDepth) / (maxDepth - minDepth).*/
	const double inputMinDepth = depths->depth_range[0];
	const double inputMaxDepth = depths->depth_range[1];

	/* Our depth range.*/
	const double minDepth;
	const double maxDepth;

	float *result = malloc(sizeof(float) * depths->x * depths->y);

	int width = depths->x;
	int height = depths->y;

	float *sourceDepths = depths->depths;
	for (int x = 0; x < width; x++){
		for (int y = 0; y < height; y++){
			float value = 0;
			/* Apply kernel to current sample of the depth map*/
			for (int kx = 0; kx < kernelSize; kx++){
				for (int ky = 0; ky < kernelSize; ky++){
					int sampledX = kx + x + kernelOffset;
					int sampledY = ky + y + kernelOffset;

					/* Bound checking.*/
					bool valid = sampledX > 0 && sampledY > 0 && sampledX < width && sampledY < height;

					value += valid ? kernel[kx][ky] * sourceDepths[sampledX + sampledY * width] : 0;
				}
			}
			result[x + y * width] = value > threshold ? value : 0;
		}
	}

	bool *depthContinuityMap = malloc(sizeof(bool) * depths->x * depths->y);

	/* Now go and filter out sharp corners.  We remove pixels at sharp corners by
	 * repeatedly removing pixels with only a single depth continuous neighbour.*/

	/* PERFORMANCE NOTE:  Improve performance here by identifying pixels that were
	 * removed in the first iteration, and then only applying the algorithm its'
	 * neighbours in subsequent itertions.*/

	const int initalRemovedPixelBufferSize = 100;

	SilhouettePixel *removedPixels = malloc(sizeof(SilhouettePixel) * initalRemovedPixelBufferSize);
	int removedPixelsBufferSize = 0;
	int maxRemovedPixelBufferSize = initalRemovedPixelBufferSize;
	bool firstLoop = true;

	/* Do first scan (we don't have an index of trouble pixels yet).*/
	for (int x = 0; x < width; x++){
		for (int y = 0; y < height; y++){

			int numberOfContinuousNeighbours = 0;

			/* Determine depth-continous neigbours for each neighbour.*/
			for (int nx = x - 1; nx < x + 2; nx++){
				for (int ny = y - 1; ny < y + 2; ny++){
					/* Dont match self*/
					if (nx == x && ny == y){
						continue;
					}

					const bool nValid = nx > 0 && ny > 0 && ny < width && ny < height
						&& result[nx + ny * width] > 0;

					if (!nValid){
						continue;
					}

					/* Determine if the neighbour has at least one continuity.*/
					int neighbourContinuities = 0;
					const float neighbourDepth = result[nx + ny * width];

					for (int cx = nx - 1; cx < x + 2; cx++){
						for (int cy = ny - 1; cy < y + 2; cy++){
							/* Dont match self*/
							if (nx == cx && ny == cy){
								continue;
							}

							const bool cValid = cx > 0 && cy > 0 && cy < width && cy < height
								&& result[cx + cy * width] > 0;
							const float neighbourNeighbourDepth = result[nx + ny * width];
							if ((neighbourDepth - neighbourNeighbourDepth) *
								(neighbourDepth - neighbourNeighbourDepth) < depthContinuityThresholdSquared){
								neighbourContinuities++;
								break;
							}
						}
						/* Needed to break out of multiple for loops*/
						if (neighbourContinuities > 0){
							break;
						}
					}
				}
			}

			/* Remove pixels without 2 depth-continuous neigbours.*/
			if (numberOfContinuousNeighbours < 2){
				result[x + y * width] = 0;
			}
			else {
				numActivePixels++;
			}
		}
	}

	free(removedPixels);

	return result;
}

/* Takes a the filtered depth buffer and returns a polyline that represents the shape
 * near the sihouette near user's stroke.
 * The returned polyline is simplified using the Douglas-Peucker algorithm
 * and the parts of the sihouette to far from the user's cursor is filtered using
 * a proximity based filter */
static void extractEdgesAsPolyLines(SketchMorphTool_OpData *data, float *simplifiedCoordBuffer, int *indicesBuffer, int *indicesListSize){

	Silhouette *s = data->Silhouette;

	SketchPoint *pointBuffer = s->pointBuffer;
	float *indicesBuffer = s->indicesBuffer;
	float *indicesListSize = s->indicesBuffer;

	const ViewDepths *depths = data->depthBuffer;
	const int width = depths->w;
	const int height = depths->h;

	/* The depth of two adjacent pixel in a line must be at max this*/
	const float depthContinuityThresholdSquared = 0.005;

	int numActivePixels = 0;
	float *filteredDepthMap = preprocessDepthMap(depths, depthContinuityThresholdSquared, &numActivePixels);

	/* coordinateBuffer stores a sequence of points that are accessed sequentially by
	 * indicesBuffer.
	 * indicesBuffer is a dynamic array that stores the number of pixels in each line.
	 * Note:  This allows us to avoid recopying because we know the number of points in
	 * coordinatebuffer. */

	const int initialindicesBufferSize = numActivePixels > 60 ? numActivePixels / 30 : 2;

	SilhouettePixel* coordinateBuffer = malloc(numActivePixels * 2 * sizeof(SilhouettePixel));
	indicesBuffer = malloc(initialindicesBufferSize * sizeof(int));

	int coordinateIndex = 0;
	int indicesIndex = 0;

	int indicesBufferSize = initialindicesBufferSize;

	/* To transform pixel lines to polylines:
	 * 1.  We iterate through the until we find a pixel.
	 * 2.  We trace that pixel to one end by continuously selecting a pixel adjacent to the last
	 *	   (preferring ones that are parallel to the current line) until we cannot find adjacent pixels.
	 * 3.  We reverse that part of the coordinate buffer.
	 * 4.  We apply the previous procedure to the other half of the line.
	 * 5.  We add the number of coordinates in the current line to the index buffer.*/
	for (int x = 0; x < width; x++){
		for (int y = 0; y < height; y++){

			/* We store the position of the last-used pixel for calculating direction*/
			SilhouettePixel currentPos = { -1, -1 };
			float lastDepthValue = filteredDepthMap[x + y * width];

			/* Depth value of 0 means that pixel did not meet the threshold.*/
			if (lastDepthValue != 0){

				/* Handle the second pixel, which is different than subsequent ones since it has no direction.*/
				for (int i = 0; i < 7; i++){
					SilhouettePixel neighbour = { x + offsetMap[i * 2], y + offsetMap[i * 2 + 1] };

					bool outOfBounds = neighbour.x <= 0 || neighbour.y <= 0 || neighbour.x >= width || neighbour.y >= height;
					if (outOfBounds){
						continue;
					}

					float depthDiff = lastDepthValue - filteredDepthMap[neighbour.x + neighbour.y * width];

					if (depthDiff * depthDiff < depthContinuityThresholdSquared){
						filteredDepthMap[neighbour.x + neighbour.y * width] = 0;

						currentPos = neighbour;
						coordinateBuffer[coordinateIndex] = currentPos;
						coordinateIndex++;
					}
				}

				if (currentPos.x == -1){
					/* No valid 2nd pixels found. This probably should not happen, but might occur in edge cases (?).*/
					continue;
				}

				/*
				 * Is added when accessing the priority mask to get the correct index.
				 * [0][1][2]
				 * [3]   [4]
				 * [5]   [6]
				 * This works because 2/2 and 3/2 both truncate to 1.
				 */
				int priorityMaskOffset = (x - currentPos.x + 1) + (y - currentPos.y + 1) * 2 + (y / 2);

				bool processingFirstHalf = true;

				/* Trace the rest of the line.*/
				for (int currentLinePixelCount = 2;; currentLinePixelCount++){
					lastDepthValue = filteredDepthMap[currentPos.x + currentPos.y * width];
					currentPos.x = -1;

					int bestPriorityValue = WINT_MAX;

					for (int i = 0; i < 7; i++){
						SilhouettePixel neighbour = { x + offsetMap[i * 2], y + offsetMap[i * 2 + 1] };

						if (neighbour.x <= 0 || neighbour.y <= 0 || neighbour.x >= width || neighbour.y >= height){
							continue;
						}

						float depthDiff = lastDepthValue - filteredDepthMap[neighbour.x + (neighbour.y)* width];
						if (depthDiff * depthDiff < depthContinuityThresholdSquared){

							/* Remove all considered pixels.  If you only remove the ones that are included
							 * in the current line then you can get a weird snaking pattern at the end of lines. */
							filteredDepthMap[neighbour.x] = 0;

							int maskValue = priorityMask[i + priorityMaskOffset];
							if (maskValue < bestPriorityValue){
								currentPos = neighbour;
								bestPriorityValue = maskValue;
							}
						}
					}

					/* Hit end of line*/
					if (currentPos.x = -1){
						if (processingFirstHalf){
							/* Reverse the just-filled part of the coordinate buffer in-place*/
							SilhouettePixel swap;

							for (int p = 0; p < currentLinePixelCount / 2; ++p){
								swap = coordinateBuffer[coordinateIndex - currentLinePixelCount + p];
								coordinateBuffer[coordinateIndex - currentLinePixelCount + p] = coordinateBuffer[coordinateIndex - p];
								coordinateBuffer[coordinateIndex - p] = swap;
							}

							/* Set the direction about the central pixel for the line to run
							 * in the opposite direction.  Note that the line already has
							 * at least two pixels..*/
							lastDepthValue = filteredDepthMap[x + y * width];
							SilhouettePixel secondLast = coordinateBuffer[coordinateIndex - 2];

							priorityMaskOffset = (x - secondLast.x + 1) + (y - secondLast.y + 1) * 2 + (y / 2);
							processingFirstHalf = false;
							continue;
						}
						else{
							/* Finish current line by putting size in index */
							if (indicesIndex >= indicesBufferSize){
								float* newIndices = malloc(indicesBufferSize * 2);
								memcpy(indicesIndex, indicesBufferSize);
								free(indicesIndex);
								indicesIndex = newIndices;
							}
							indicesBuffer[indicesIndex] = currentLinePixelCount + 1;
							indicesIndex++;

							break;
						}
					}

					coordinateBuffer[coordinateIndex] = currentPos;
					coordinateIndex++;
				}
			}
		}
	}

	/* Maximum squared of any point from the generated simplified line.*/
	const float maxError = 8;
	int *newCoordBufferSize;
	int *newIndicesBufferSize;

	simplifyLines(coordinateBuffer, indicesBuffer, indicesIndex, maxError * maxError);
	proximityCullSilhouette(data, coordinateBuffer, indicesBuffer, newCoordBufferSize, newIndicesBufferSize, indicesIndex);

	/* There is a bunch of shit all over the end of these buffers to remove*/
	SketchPoint *simplifiedPointBuffer;
	malloc(simplifiedPointBuffer, (*newCoordBufferSize) * sizeof(SketchPoint));
	memcpy(simplifiedPointBuffer, coordinateBuffer, sizeof(float) * (*newCoordBufferSize));

	int *simplifiedPointBuffer;
	malloc(simplifiedPointBuffer, (*newIndicesBufferSize) * sizeof(int));
	memcpy(simplifiedPointBuffer, coordinateBuffer, sizeof(float) * (*newCoordBufferSize));

	data->Silhouette->pointBuffer = simplifiedPointBuffer;
	data->Silhouette->indicesBuffer = simplifiedCoordBuffer;
	data->Silhouette->numberOfIndices = newIndicesBufferSize;

	free(coordinateBuffer);
	free(filteredDepthMap);
}

/* todo*/
static void proximityCullSilhouette(SketchMorphTool_OpData *data, SketchPoint *coordinateBuffer, int *indicesBuffer, int *newCoordBufferSize, int *newIndicesBufferSize, const int indiceBufferSize){
	/* */
}


/* Simplify the set of lines using Douglas Peucker algorithm.
 * Allocates memory for SimplfiedBuffer*/
static void *simplifySilhouette(SilhouettePixel *coordinateBuffer, int *indicesBuffer, const int indiceBufferSize, const int maxDistanceSquared){

	BLI_assert(sizeof(SketchPoint) <= sizeof(SilhouettePixel));

	/* Using a stack-based implmenetation*/
	FloatPairLinked *stackHead = NULL;

	/* We take take things from coordinatebuffer and add them to coordinateBuffer float.
	 * This is not needed but helps with readability. */
	SketchPoint* coordinateBufferFloat = (SketchPoint  *)coordinateBuffer;

	/* We work from left to right on the stroke overwriting our unsimplified data.
	* At the end we then copy simpified data into a new array.*/
	int simplifyIndex = 0;
	int strokeIndex = 0;

	for (int i = 0; i < indiceBufferSize; i++){
		/* Stack should be empty at this point*/
		stackHead = malloc(sizeof(FloatPairLinked));
		stackHead->a = strokeIndex;
		stackHead->b = strokeIndex + indicesBuffer[i];

		while (stackHead != NULL){
			int beginning = stackHead->a;
			int end = stackHead->b;

			/* Line segment has two points and cannot be simplified futher. */
			if (end - beginning == 1){
				SketchPoint beginningPoint = { (float)coordinateBuffer[beginning].x, (float)coordinateBuffer[beginning].y };
				coordinateBufferFloat[simplifyIndex] = beginningPoint;
				simplifyIndex++;
				SketchPoint endPoint = { (float)coordinateBuffer[beginning].x, (float)coordinateBuffer[beginning].y };
				coordinateBufferFloat[simplifyIndex] = endPoint;
				simplifyIndex++;
			}

			SketchPoint startPoint = { (float)coordinateBuffer[beginning].x, (float)coordinateBuffer[beginning].y };
			SketchPoint endPoint = { (float)coordinateBuffer[end].x, (float)coordinateBuffer[end].y };

			/* Find implicit coefficients (standard form) of the current line.
			 * 0 = x - y * (e_x - s_x)/(e_y - s_y) - s_y * (e_x - s_x)/(e_y - s_y) - s_x  */
			const float a = 1;
			const float b = -(endPoint.x - startPoint.x) / (endPoint.y - startPoint.y);
			const float c = -startPoint.y * (endPoint.x - startPoint.x) / (endPoint.y - startPoint.y) - startPoint.x;
			const float denom = a*a + b*b;

			/* Find the furthest point from line going from beginning->end on the current line */
			float maxDist = 0;
			int maxDistIndex = 0;

			for (int t = beginning + 1; i < end - 1; i++){
				int temp = (a * coordinateBuffer[t].x + b * coordinateBuffer[t].y + c);
				float distSquared = temp*temp / denom;
				if (distSquared > maxDist){
					maxDistIndex = t;
					maxDist = distSquared;
				}
			}

			if (maxDist < maxDistanceSquared){
				for (int t = beginning; i < end; i++){
					SketchPoint pos = { coordinateBuffer[t].x, coordinateBuffer[t].y };
					coordinateBufferFloat[simplifyIndex] = pos;
					simplifyIndex++;
				}

				FloatPairLinked *next = stackHead->next;
				free(stackHead);
				stackHead = next;
				continue;
			}
			else{
				/* Split the line at the furthest vertext from the line and run the procedure
				 * on each of the sublines.*/
				FloatPairLinked *next = stackHead->next;
				FloatPairLinked *leftSide = malloc(sizeof(FloatPairLinked));
				FloatPairLinked *rightSide = malloc(sizeof(FloatPairLinked));
				leftSide->a = beginning;
				leftSide->b = maxDistIndex - 1;
				leftSide->next = rightSide;
				rightSide->a = maxDistIndex;
				rightSide->b = end;
				rightSide->next = next;
				free(stackHead);
				stackHead = leftSide;
				continue;
			}
		}
		/*Overwrite indices buffer*/
		if (i == 0){
			indicesBuffer[i] = simplifyIndex;
		}
		else{
			indicesBuffer[i] = simplifyIndex - indicesBuffer[i - 1];
		}
		strokeIndex += indicesBuffer[i];
	}
}

/****************************************************************
* SECTION ON MATCHING USER STROKE TO THE CORRECT SILHOUETTE LINE*
*****************************************************************/

/* Calcula*/

/* Scores the similarity of the given user sketch and silhouette
 * edge.  Based off the length of the pattern and the mean squared
 * error.
 */
static float sketchSilhouetteScore(float *sketchPositions, float *silhouettePositions,
	float *sketchTurningAngles, float *silhouetteTurningAngles,
	float rightShift, float upShift, float stretch,
	int sketchLength, int silhouetteLength){

}


/* Determines the similarity of the line with each candidate line.
 * Works in the way described here:  http://robotics.stanford.edu/~scohen/pssp-soda-final.pdf */
static float *determineSimilarities(SketchMorphTool_OpData *op){

	Silhouette *sil = op->Silhouette;
	SketchStroke* stroke = op->pointList;

	SketchPoint *edges = sil->pointBuffer;
	int *indices = sil->indicesBuffer;
	int numEdges = sil->numberOfIndices;
	int numPointsSil = sil->numberOfPoints;

	SketchPoint *sketchPoints = stroke->pointData;
	int numSketchPoints = stroke->curSize;

	float *sketchTurningAngles = malloc(sizeof(float) * numSketchPoints);
	float *silhouetteTurningAngles = malloc(sizeof(float) * numSketchPoints);

	float *sketchSegmentLengths = malloc(sizeof(float) * numSketchPoints);
	float *sihouetteSegmentLengths = malloc(sizeof(float) * numSketchPoints);

	/* Compute turning angles and squared segment lengths for the user's stroke.*/
	for (int i = 0; i < numSketchPoints - 1; i++){

		/* This should be numerically stable and determinant at anything
		* where both components are not 0 which is not possible here */
		SketchPoint a = sketchPoints[i];
		SketchPoint b = sketchPoints[i + 1];

		sketchTurningAngles[i + 1] = sketchTurningAngles[i] + atan2(b.y, b.x) - atan2(a.y, a.x);
		sketchSegmentLengths[i + 1] = sketchSegmentLengths[i] + (a.x - b.x) * (a.x - b.x) + (a.y - b.y) + (a.y - b.y);
	}

	/* Compute turning angles and squared segment lengths for the silhouette edges */
	int indicesIndex = 0;
	for (int i = 0; i < numEdges; i++){
		int numTurningPoints = indices[i] - 1;
		for (int p = 0; p < numTurningPoints; p++){

			/* This should be numerically stable and determinant at anything
			* where both components are not 0 which is not possible here */
			SketchPoint a = sketchPoints[indicesIndex + i];
			SketchPoint b = sketchPoints[indicesIndex + i + 1];

			silhouetteTurningAngles[i + 1] = silhouetteTurningAngles[i] + atan2(b.y, b.x) - atan2(a.y, a.x);
			sihouetteSegmentLengths[i + 1] = silhouetteTurningAngles[i] + (a.x - b.x) * (a.x - b.x) + (a.y - b.y) + (a.y - b.y);
		}
		indicesIndex += numTurningPoints + 1;
	}

	free(sketchTurningAngles);
	free(silhouetteTurningAngles);

	free(sketchSegmentLengths);
	free(sihouetteSegmentLengths);
}

/* Performs the algorithm and then flushes all the buffers*/
static void sketch_morph_finish(SketchMorphTool_OpData *op){

}

/* Cancels the action and flushes all buffers*/
static void sketch_morph_cancel(SketchMorphTool_OpData *op){

}

/* Resets the objects.  Used for when we want to draw multiple items.*/
static void sketch_morph_reset(SketchMorphTool_OpData *data){
	free(data->pointList->pointData);
	data->pointList->pointData = NULL;
}
