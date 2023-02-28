package org.firstinspires.ftc.teamcode.library.vision.powerplay

import org.firstinspires.ftc.teamcode.library.vision.base.ImageResolution
import org.firstinspires.ftc.teamcode.library.vision.base.ResolutionPipeline
import org.firstinspires.ftc.teamcode.library.vision.base.coerceIn
import org.firstinspires.ftc.teamcode.library.vision.base.times
import org.firstinspires.ftc.teamcode.library.vision.powerplay.SignalVisionConstants.*

import org.opencv.core.*
import org.opencv.core.Core.*
import org.opencv.imgproc.Imgproc
import org.opencv.imgproc.Imgproc.*


class SignalVisionPipeline() : ResolutionPipeline() {

    // Public variable allowing OpModes to access contour details
    var contourResults: List<ContourResult?> = List(3) { null }

    // The mat that we will analyze, alone with the channel number we need to extract from HSV
    private var subMat: Mat = Mat()
    private var hlsMat: Mat = Mat()
    private var thresholdResults = Array(3) { Mat() }

    override var resolution : ImageResolution = ImageResolution.R_720x480

    override fun processFrame(input: Mat): Mat {
        if (tracking) {

            // Blur the image for greater contour recognition accuracy
            blur(input, input, Size(5.0, 10.0))

            subMat = input.submat((CUTOFF_TOP * input.rows()).toInt(), (CUTOFF_BOTTOM * input.rows()).toInt(), (CUTOFF_LEFT * input.cols()).toInt(), (CUTOFF_RIGHT * input.cols()).toInt())

            // Convert our input, in RGB format, to HLS (hue, luminosity, saturation)
            cvtColor(subMat, hlsMat, COLOR_RGB2HLS)

            val contourHueLowerBound = arrayOf(
                    CONTOUR_HUE_LOWER_BOUND_RED,
                    CONTOUR_HUE_LOWER_BOUND_GREEN,
                    CONTOUR_HUE_LOWER_BOUND_BLUE
            )
            val contourHueUpperBound = arrayOf(
                    CONTOUR_HUE_UPPER_BOUND_RED,
                    CONTOUR_HUE_UPPER_BOUND_GREEN,
                    CONTOUR_HUE_UPPER_BOUND_BLUE
            )

            val contourLumLowerBound = arrayOf(
                    CONTOUR_LUM_LOWER_BOUND_RED,
                    CONTOUR_LUM_LOWER_BOUND_GREEN,
                    CONTOUR_LUM_LOWER_BOUND_BLUE
            )
            val contourLumUpperBound = arrayOf(
                    CONTOUR_LUM_UPPER_BOUND_RED,
                    CONTOUR_LUM_UPPER_BOUND_GREEN,
                    CONTOUR_LUM_UPPER_BOUND_BLUE
            )

            val contourSatLowerBound = arrayOf(
                    CONTOUR_SAT_LOWER_BOUND_RED,
                    CONTOUR_SAT_LOWER_BOUND_GREEN,
                    CONTOUR_SAT_LOWER_BOUND_BLUE
            )
            val contourSatUpperBound = arrayOf(
                    CONTOUR_SAT_UPPER_BOUND_RED,
                    CONTOUR_SAT_UPPER_BOUND_GREEN,
                    CONTOUR_SAT_UPPER_BOUND_BLUE
            )

            // Threshold the HLS mat to only include objects within desired HLS range
            for (i in 0..2) {
                inRange(
                        hlsMat,                             // original mat
                        Scalar(                             // lower bound for threshold
                                contourHueLowerBound[i],
                                contourLumLowerBound[i],
                                contourSatLowerBound[i]),
                        Scalar(                             // upper bound for threshold
                                contourHueUpperBound[i],
                                contourLumUpperBound[i],
                                contourSatUpperBound[i]),
                        thresholdResults[i]                     // resultant mat
                )
            }

            val thresholdResultsMerged = Mat()
            merge(thresholdResults.toList(),thresholdResultsMerged)

            val elementType = Imgproc.CV_SHAPE_RECT;
            val kernelSize = CONTOUR_DILATION_KSIZE;
            val element = getStructuringElement(
                    elementType, Size(2 * kernelSize + 1, 2 * kernelSize + 1),
                    Point(kernelSize, kernelSize)
            )
            for (i in 0..2) {
                dilate(
                        thresholdResults[i],                    // original mat
                        thresholdResults[i],                    // resultant mat - just overwrite the original
                        element                             // iterations - more of these means more erosion
                )
            }

            val drawingMat = when(MAT_OUTPUT_NUM) {
                0       -> input
                1       -> hlsMat
                2       -> thresholdResults[0]
                3       -> thresholdResults[1]
                4       -> thresholdResults[2]
                else    -> thresholdResultsMerged
            }

            val contours = List(3) { emptyList<MatOfPoint>().toMutableList() }
            val hierarchy = Array(3) { Mat() }
            for (i in 0..2) {
                findContours(
                        thresholdResults[i],            // original mat
                        contours[i],                    // resultant list of contours
                        hierarchy[i],                   // hierarchy of contours
                        RETR_TREE,                      // contour retrieval mode
                        CHAIN_APPROX_SIMPLE             // contour approximation method
                )
            }

            val contoursCmpltd = contours.mapIndexed { _, contour ->
                contour.mapIndexed map@{ index, matOfPoint ->
                    drawContours(
                            drawingMat,
                            contour,
                            index,
                            Scalar.all(150.0),
                            3
                    )

                    var minX = Int.MAX_VALUE
                    var minY = Int.MAX_VALUE
                    var maxX = Int.MIN_VALUE
                    var maxY = Int.MIN_VALUE
                    for (point in matOfPoint.toArray()) {
                        if (point.x < minX) minX = point.x.toInt()
                        if (point.y < minY) minY = point.y.toInt()
                        if (point.x > maxX) maxX = point.x.toInt()
                        if (point.y > maxY) maxY = point.y.toInt()
                    }

                    println("CONTOUR $index min=($minX, $minY) max=($maxX, $maxY)")

                    return@map ContourResult(
                            Point(minX.toDouble(), minY.toDouble()),
                            Point(maxX.toDouble(), maxY.toDouble()))
                }
            }
            val resCntur = contoursCmpltd.mapIndexed { _, contour ->
                contour.filter { it.width > CONTOUR_ENTITY_MINWIDTH * resolution.scale && it.min.y < input.rows() * (0.70) }
                    .maxByOrNull { it.area }
            }
            this.contourResults = resCntur

            addLabels(drawingMat)
            return drawingMat
        }
        else {
            addLabels(input)
            return input
        }
    }

    inner class ContourResult
    constructor(val min: Point, val max: Point)
    {
        val width = max.x - min.x
        val height = max.y - min.y
        val ratio = width.toDouble() / height
        val area = width * height
        val valid = ratio in RATIO_LOWER_BOUND..RATIO_UPPER_BOUND

        val standardized: ContourResult
        get() = ContourResult(
            Point(min.x / this@SignalVisionPipeline.resolution.width,
                min.y / this@SignalVisionPipeline.resolution.height),
            Point(max.x / this@SignalVisionPipeline.resolution.width,
                max.y / this@SignalVisionPipeline.resolution.height),
        )
    }

    private fun inverseColorAtPoint(mat: Mat, point: Point): DoubleArray {
        val newPoint = point.coerceIn(mat)
        return mat.get(newPoint.y.toInt(), newPoint.x.toInt()) // Get color at this point
                .map { 255 - it }                              // For each channel, invert the color
//                .dropLast(1)
                .toDoubleArray()                               // Re-convert to DoubleArray
    }


    private fun addLabels(mat: Mat) {

        // Place text representing the team name (at the top)
        val teamNameStartPoint = Point(5.0, 30.0)
                .times(resolution.scale)
                .coerceIn(mat)
        putText(mat, "Eagle Robotics Team 7373", teamNameStartPoint,
                FONT_HERSHEY_SIMPLEX, 0.5 * resolution.scale,
                Scalar(inverseColorAtPoint(mat, teamNameStartPoint)), 2)

        // Place text indicating the detector purpose (below the team name)
        val detectorNameStartPoint = Point(5.0, 50.0)
                .times(resolution.scale)
                .coerceIn(mat)
        putText(mat, "POWER PLAY SignalVision Pipeline", detectorNameStartPoint,
                FONT_HERSHEY_SIMPLEX, 0.25 * resolution.scale,
                Scalar(inverseColorAtPoint(mat, detectorNameStartPoint)), 2)

        // Below code left in-place in case specfic result should be shown on camera feed
//        putText(mat, resultText,
//                resultTextStartPoint,
//                FONT_HERSHEY_SIMPLEX, 0.6 * resolution.scale,
//                Scalar(inverseColorAtPoint(mat, resultTextStartPoint)), 2)
    }




}