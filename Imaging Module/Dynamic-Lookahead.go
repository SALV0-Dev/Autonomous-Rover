package main

import (
	"fmt"
	"image"
	"time"
	"image/color"

	pb_output "github.com/VU-ASE/pkg-CommunicationDefinitions/v2/packages/go/outputs"
	pb_systemmanager_messages "github.com/VU-ASE/pkg-CommunicationDefinitions/v2/packages/go/systemmanager"
	"google.golang.org/protobuf/proto"

	servicerunner "github.com/VU-ASE/pkg-ServiceRunner/v2/src"
	zmq "github.com/pebbe/zmq4"
	"gocv.io/x/gocv"

	"github.com/rs/zerolog/log"
)

type SliceDescriptor struct {
	Start int // Start index of the array
	End   int // End index of the array
}

// This function scans the slice for points that are full white (non-black) (after thresholding)
// It returns an array of descriptions of the consecutive white points
// r.i.p. mrbuggy :(
func getConsecutiveWhitePointsFromSlice(imageSlice *gocv.Mat) []SliceDescriptor {
	res := []SliceDescriptor{}

	var currentConsecutive *SliceDescriptor = nil

	for i := 0; i < imageSlice.Cols()-1; i++ {
		currentByte := imageSlice.GetVecbAt(0, i)[0]

		// byte(0) indicates black, byte(255) indicates white
		if currentByte != byte(0) {
			// Current point is a white point. Is there already a consecutive array?
			if currentConsecutive == nil {
				// No, create a new one
				currentConsecutive = &SliceDescriptor{
					Start: i,
					End:   i,
				}
			} else {
				// Yes, extend the current one
				currentConsecutive.End = i
			}
		} else {
			// Current point is black. Is there a consecutive array?
			if currentConsecutive != nil {
				// Yes, add it to the result, if it's at minimum 1 pixel wide
				if currentConsecutive.End-currentConsecutive.Start > 0 {
					res = append(res, *currentConsecutive)
				}
				currentConsecutive = nil
			}
		}
	}

	// We reached the right edge of the image. If there is a consecutive array, add it to the result
	if currentConsecutive != nil && currentConsecutive.End-currentConsecutive.Start > 0 {
		res = append(res, *currentConsecutive)
	}

	return res
}

// This function takes an array of slice descriptors and finds the one with the most consecutive white pixels
// It returns nil if no such slice is found
func getLongestConsecutiveWhiteSlice(sliceDescriptors []SliceDescriptor) *SliceDescriptor {
	if len(sliceDescriptors) == 0 {
		return nil
	}

	longest := sliceDescriptors[0]
	for _, desc := range sliceDescriptors {
		if (desc.End - desc.Start) > (longest.End - longest.Start) {
			longest = desc
		}
	}

	log.Info().Int("longest", longest.End-longest.Start).Msg("Longest consecutive white slice")
	log.Info().Int("start", longest.Start).Int("end", longest.End).Msg("Start and end of longest consecutive white slice")

	for _, desc := range sliceDescriptors {
		log.Info().Int("start", desc.Start).Int("end", desc.End).Msg("[OPTION] and end of slice")
	}

	return &longest
}

func verticalScanUp(imageSlice *gocv.Mat) int {
	y := 479
	for y >= 0 {
		if imageSlice.GetVecbAt(y,0)[0] == 0 {
			return y
		}
		y--
	}
	return y + 1
}

func detectNewBoundary(imageSlice *gocv.Mat, prevY int) int {
	y := prevY

	if  imageSlice.GetVecbAt(prevY,0)[0] == 0 { // look for white 
		for y + 2 <= 479 { // go down until you find white
			y = y + 2 // O(n/2)
			if imageSlice.GetVecbAt(y,0)[0] == 255 {
				break
			}
		}
	} else { // look for black
		for y - 2 >= 0{ //go up until you find blakc
			y = y - 2
			if imageSlice.GetVecbAt(y,0)[0] == 0 {
				break
			}
		}
	}	
	return y
}

// Runs the program logic
func run(service servicerunner.ResolvedService, sysmanInfo servicerunner.SystemManagerInfo, tuning *pb_systemmanager_messages.TuningState) error {
	// Fetch runtime parameters
	// Fetch pipeline from tuning (statically defined in service.yaml)
	gstPipeline, err := servicerunner.GetTuningString("gstreamer-pipeline", tuning)
	if err != nil {
		log.Err(err).Msg("Failed to get gstreamer-pipeline from tuning. Is it defined in service.yaml?")
		return err
	}
	// Fetch thresholding value
	thresholdValue, err := servicerunner.GetTuningInt("threshold-value", tuning)
	if err != nil {
		return err
	}
	// Fetch width to put in gstreaqmer pipeline
	imgWidth, err := servicerunner.GetTuningInt("imgWidth", tuning)
	if err != nil {
		return err
	}
	// Fetch height to put in gstreamer pipeline
	imgHeight, err := servicerunner.GetTuningInt("imgHeight", tuning)
	if err != nil {
		return err
	}
	// Fetch image fps to put in gstreamer pipeline
	imgFps, err := servicerunner.GetTuningInt("imgFPS", tuning)
	if err != nil {
		return err
	}
	// Create the gstreamer pipeline with the fetched parameters
	gstPipeline = fmt.Sprintf(gstPipeline, imgWidth, imgHeight, imgFps)

	// Fetch address to send output to
	outputAddr, err := service.GetOutputAddress("path")
	if err != nil {
		return err
	}
	// And build publisher socket using ZMQ
	sock, err := zmq.NewSocket(zmq.PUB)
	if err != nil {
		return err
	}
	err = sock.Bind(outputAddr)
	if err != nil {
		return err
	}

	// Open video capture using gstreamer pipeline
	cam, err := gocv.OpenVideoCapture(gstPipeline)
	if err != nil {
		return err
	}
	defer cam.Close()

	// Complete images are stored in this mat
	buf := gocv.NewMat()
	defer buf.Close()

	// Y coordinate of the horizontal slice used for steering
	//constYslice  := 240

	// Initilise variables  
	start := uint8(1)
	prevDetectedBoundary := 0 
	currDetectedBoundary := 0 
	inCurves := uint8(0) // cruising and in-curve state
	curveStart := uint8(0)
	rowIndex := 0

	
	cruisingLookahead := 240

	//find best values for these 
	inCurveBoundaryThreshold := 200 // find best value for speed 0.2 - 0.4
	lowerLimitFormula := 100  
	upperLimitFormula := 170
	outOfCurve_DistanceThreshold := 150  //130
	
	

	for {
		if ok := cam.Read(&buf); !ok {
			log.Warn().Err(err).Msg("Error reading from camera")
			continue
		}
		if buf.Empty() {
			continue
		}
		imgWidth := buf.Cols()
		imgHeight := buf.Rows()

		log.Info().Int("width", imgWidth).Int("height", imgHeight).Msg("Read image")
		 
		

		// Standard Thresholding Image segmentation
		gocv.CvtColor(buf, &buf, gocv.ColorBGRToGray)
		gocv.Threshold(buf, &buf, float32(thresholdValue), 255.0, gocv.ThresholdBinary+gocv.ThresholdOtsu)
		kernel := gocv.GetStructuringElement(gocv.MorphRect, image.Pt(5, 5))
		gocv.Dilate(buf, &buf, kernel)
		gocv.Erode(buf, &buf, kernel)


		verticalSlice := buf.Region(image.Rect(imgWidth/2, 0, imgWidth/2 + 1, imgHeight))
		if start == 1 {
			currDetectedBoundary = verticalScanUp(&verticalSlice) //initiate boundary
		} else {
			currDetectedBoundary = detectNewBoundary(&verticalSlice, prevDetectedBoundary)
		}

		println("raw boundary", currDetectedBoundary)


		if inCurves == uint8(0){  
			if currDetectedBoundary >= inCurveBoundaryThreshold {	// curve boundary is "close" and it is now in curve
				rowIndex = currDetectedBoundary 
				inCurves = uint8(1)
				curveStart = uint8(1)
			} else if currDetectedBoundary > lowerLimitFormula && currDetectedBoundary < inCurveBoundaryThreshold{
				if currDetectedBoundary <= upperLimitFormula { // magic formula 100 < x < 170
					rowIndexFloat := (3.4 * float32(currDetectedBoundary)) - float32(100) // reaches max of 478 
					rowIndex = int(rowIndexFloat)
				} else {
					rowIndex = imgHeight - 2  // stays at 478
				}
			} else { // it is in straight
				rowIndex = cruisingLookahead
			}
		} else { 
			println(currDetectedBoundary, prevDetectedBoundary)
			
			if rowIndex < currDetectedBoundary && currDetectedBoundary <= cruisingLookahead{ // get "back" to 240
				rowIndex = currDetectedBoundary
			} else if currDetectedBoundary > prevDetectedBoundary + 20 {
				rowIndex = currDetectedBoundary
			}
			
			
			if curveStart == uint8(1){ 
				if currDetectedBoundary > prevDetectedBoundary + 10 { 
					rowIndex = currDetectedBoundary
				}
			}

			if currDetectedBoundary < outOfCurve_DistanceThreshold {
				println("no longer in curve")
				inCurves = uint8(0)
				rowIndex = cruisingLookahead  
			} 
			curveStart = uint8(0)
			
		}

		prevDetectedBoundary = currDetectedBoundary

		if start == uint8(1) { // no longer in start
			start = uint8(0)
		}

		println(rowIndex)

		// Take a slice that is used to steer on
		horizontalSlice := buf.Region(image.Rect(0, rowIndex, imgWidth, rowIndex+1))

		// Find the consecutive white points
		sliceDescriptors := getConsecutiveWhitePointsFromSlice(&horizontalSlice)
		// Find the longest consecutive white slice
		longestConsecutive := getLongestConsecutiveWhiteSlice(sliceDescriptors)


		if longestConsecutive == nil {
			continue
		}

		////////// Setup View for Web Ui and Saved Images //////////
		

		//Horizontal points
		startPoint := image.Pt(longestConsecutive.Start, rowIndex)
		endPoint := image.Pt(longestConsecutive.End, rowIndex)
		ErrorPoint := image.Pt(((longestConsecutive.Start + longestConsecutive.End)/2), rowIndex)
		middePoint := image.Pt(buf.Cols()/2 , rowIndex)

		//Vertical points
		VstartPoint1 := image.Pt(buf.Cols()/2, currDetectedBoundary)
		VendPoint := image.Pt(buf.Cols()/2, buf.Rows())
		VrowIndexPoint := image.Pt((buf.Cols()/2), rowIndex)

		gocv.CvtColor(buf, &buf, gocv.ColorGrayToBGR)

		//Horizontal lines 
		if (longestConsecutive.Start + longestConsecutive.End)/2 <= buf.Cols()/2 {
			gocv.Line(&buf, startPoint, ErrorPoint, color.RGBA{R:128, G:133, B:133, A:0}, 1)
			gocv.Line(&buf, ErrorPoint, middePoint, color.RGBA{R:255, G:0, B:0, A:0}, 2) // Error
			gocv.Line(&buf, middePoint, endPoint, color.RGBA{R:128, G:133, B:133, A:0}, 1)
		}else {
			gocv.Line(&buf, startPoint, middePoint, color.RGBA{R:128, G:133, B:133, A:0}, 1)
			gocv.Line(&buf, middePoint, ErrorPoint, color.RGBA{R:255, G:0, B:0, A:0}, 2) // Error
			gocv.Line(&buf, ErrorPoint, endPoint, color.RGBA{R:128, G:133, B:133, A:0}, 1)
		}
		//Vertical lines
		gocv.Line(&buf, VstartPoint1, VrowIndexPoint, color.RGBA{R:128, G:133, B:133, A:0}, 1) 
		gocv.Line(&buf, VendPoint, VrowIndexPoint, color.RGBA{R:0, G:255, B:0, A:0}, 2) //height

		gocv.Circle(&buf, VstartPoint1, 5, color.RGBA{R:220, G:0, B:200, A:0}, -1)

		gocv.IMWrite("/home/debix/myFiles/image.jpg", buf)
		
		//////////////////////////////////////////////////

		
		sliceY := uint32(rowIndex)
		// Create a canvas that can be drawn on
		canvasObjects := make([]*pb_output.CanvasObject, 0)
		// Draw points where the longest consecutive slice starts, ends and the middle
		if longestConsecutive != nil {
			middleX := (longestConsecutive.Start + longestConsecutive.End) / 2
			//prevError = middleX

			// Draw start
			canvasObjects = append(canvasObjects, &pb_output.CanvasObject{
				Object: &pb_output.CanvasObject_Circle_{
					Circle: &pb_output.CanvasObject_Circle{
						Center: &pb_output.CanvasObject_Point{
							X: uint32(longestConsecutive.Start),
							Y: sliceY,
						},
						Radius: 1,
					},
				},
			})
			// Draw end
			canvasObjects = append(canvasObjects, &pb_output.CanvasObject{
				Object: &pb_output.CanvasObject_Circle_{
					Circle: &pb_output.CanvasObject_Circle{
						Center: &pb_output.CanvasObject_Point{
							X: uint32(longestConsecutive.End),
							Y: sliceY,
						},
						Radius: 1,
					},
				},
			})
			// Draw middle
			canvasObjects = append(canvasObjects, &pb_output.CanvasObject{
				Object: &pb_output.CanvasObject_Circle_{
					Circle: &pb_output.CanvasObject_Circle{
						Center: &pb_output.CanvasObject_Point{
							X: uint32(middleX),
							Y: sliceY,
						},
						Radius: 1,
					},
				},
			})
		}

		canvas := pb_output.Canvas{
			Objects: canvasObjects,
			Width:   uint32(imgWidth),
			Height:  uint32(imgHeight),
		}

		// used for JPEG compression
		var compressionParams [2]int
		compressionParams[0] = gocv.IMWriteJpegQuality
		compressionParams[1] = 30 // the quality
		// Convert the image to JPEG bytes
		imgBytes, err := gocv.IMEncodeWithParams(".jpg", buf, compressionParams[:])
		if err != nil {
			log.Err(err).Msg("Error encoding image")
			return err
		}

		// Create the trajectory, (currently it is just the middle of the longest consecutive slice)
		trajectory_points := make([]*pb_output.CameraSensorOutput_Trajectory_Point, 0)
		if longestConsecutive != nil {
			middleX := (longestConsecutive.Start + longestConsecutive.End) / 2
			trajectory_points = append(trajectory_points, &pb_output.CameraSensorOutput_Trajectory_Point{
				X: uint32(middleX),  // add +/80 for left right lane positioning
				Y: sliceY,
			})

			log.Debug().Int("x", middleX).Msg("Trajectory added")
		} else {
			log.Debug().Msg("No trajectory added")
		}

		// Make it a sensor output
		output := pb_output.SensorOutput{
			SensorId:  25,
			Timestamp: uint64(time.Now().UnixMilli()),
			SensorOutput: &pb_output.SensorOutput_CameraOutput{
				CameraOutput: &pb_output.CameraSensorOutput{
					DebugFrame: &pb_output.CameraSensorOutput_DebugFrame{
						Jpeg:   imgBytes.GetBytes(),
						Canvas: &canvas,
					},
					Trajectory: &pb_output.CameraSensorOutput_Trajectory{
						Points: trajectory_points,
						Width:  640,
						Height: 480,
					},
					Flags: 0,
				},
			},
		}
		outputBytes, err := proto.Marshal(&output)
		if err != nil {
			log.Err(err).Msg("Error marshalling sensor output")
			continue
		}

		// Send the image
		i, err := sock.SendBytes(outputBytes, 0)
		if err != nil {
			log.Err(err).Msg("Error sending image")
			return err
		}

		log.Debug().Int("bytes", i).Msg("Sent image")
		horizontalSlice.Close() // avoid memory leaks
	}
}

func onTuningState(tuningState *pb_systemmanager_messages.TuningState) {
	log.Info().Msg("Received tuning state, but I don't care lol. You can't stop me. I'm a train. Choo choo.")
}

// Used to start the program with the correct arguments
func main() {
	servicerunner.Run(run, onTuningState, nil, false)
}
