package main

import (
	"time"
	"fmt"

	pb_outputs "github.com/VU-ASE/pkg-CommunicationDefinitions/v2/packages/go/outputs"
	pb_systemmanager_messages "github.com/VU-ASE/pkg-CommunicationDefinitions/v2/packages/go/systemmanager"
	servicerunner "github.com/VU-ASE/pkg-ServiceRunner/v2/src"
	zmq "github.com/pebbe/zmq4"
	"github.com/rs/zerolog/log"
	pid "go.einride.tech/pid"
	"google.golang.org/protobuf/proto"

	keyboard "github.com/eiannone/keyboard"
)

func run(
	service servicerunner.ResolvedService,
	sysMan servicerunner.SystemManagerInfo,
	initialTuning *pb_systemmanager_messages.TuningState) error {

	// Get the address of trajectory data output by the imaging module
	imagingTrajectoryAddress, err := service.GetDependencyAddress("imaging", "path")
	if err != nil {
		return err
	}

	// Get the address to which to send the decision data for the actuator to use
	decisionAddress, err := service.GetOutputAddress("decision")
	if err != nil {
		return err
	}

	// Create a socket to send decision data on
	outputSock, err := zmq.NewSocket(zmq.PUB)
	if err != nil {
		return err
	}
	err = outputSock.Bind(decisionAddress)
	if err != nil {
		return err
	}

	// Create a socket to receive trajectory data on
	imagingSock, err := zmq.NewSocket(zmq.SUB)
	if err != nil {
		return err
	}
	err = imagingSock.Connect(imagingTrajectoryAddress)
	if err != nil {
		return err
	}
	err = imagingSock.SetSubscribe("") // Subscribe to all messages
	if err != nil {
		return err
	}

	// Get PID tuning values
	kp, err := servicerunner.GetTuningFloat("kp", initialTuning)
	if err != nil {
		return err
	}
	ki, err := servicerunner.GetTuningFloat("ki", initialTuning)
	if err != nil {
		return err
	}
	kd, err := servicerunner.GetTuningFloat("kd", initialTuning)
	if err != nil {
		return err
	}

	// Get speed to use
	/*speed1, err := servicerunner.GetTuningFloat("speed", initialTuning)
	if err != nil {
		return err
	}*/

	// Get the desired trajectory point
	desiredTrajectoryPoint, err := servicerunner.GetTuningInt("desired-trajectory-point", initialTuning)
	if err != nil {
		return err
	}

	// Initialize pid controller
	pidController := pid.Controller{
		Config: pid.ControllerConfig{
			ProportionalGain: float64(kp),
			IntegralGain:     float64(ki),
			DerivativeGain:   float64(kd),
		},
	}

	err = keyboard.Open()
	if err != nil {
		return err
	}

	speed := float32(0.2)
	speedIncrement := float32(0.05)
	kpIncrement := float32(0.00005)
	kdIncrement := float32(0.0001)

	go func() { //go routine for keyboard input
		defer keyboard.Close()
		for {
			char, key, err := keyboard.GetKey()
			char ++
			if err!= nil {
				log.Err(err).Msg("Error getting key press")
				continue
			}
			if key==keyboard.KeyArrowUp {
				speed += speedIncrement
				if speed > 0.5 {
					speed = 0.5
				}
				fmt.Println("Speed increased to:", speed)
			} else if key==keyboard.KeyArrowDown {
				speed -= speedIncrement
				if speed < 0 {
					speed = 0
				}
				fmt.Println("Speed decreased to:", speed)
			} else if key == keyboard.KeyEsc {
				break							
			} else if key == keyboard.KeyCtrlA{ //increase kp
				kp += kpIncrement
				if kp > 1 {
					kp = 1
				}
				fmt.Println("kp increased to:", kp) 
			} else if key == keyboard.KeyCtrlS { // decrease kp
				kp -= kpIncrement
				if kp < 0 {
					kp = 0
				}
				fmt.Println("kp decreased to:", kp)
			} else if key == keyboard.KeyCtrlD { // increase kd
				kd += kdIncrement
				if kd > 1 {
					kd = 1
				}
				fmt.Println("kd increased to:", kd) 
			} else if key == keyboard.KeyCtrlH{ // decrease kd
				kd -= kdIncrement
				if kd < 0 {
					kd = 0
				}
				fmt.Println("kd decreased to:", kd)
			}

			time.Sleep(time.Millisecond * 100) // this should delay high CPU usage in case
		}
	}()

	// Main loop, subscribe to trajectory data and send decision data
	for {
		// Receive trajectory data
		sensorBytes, err := imagingSock.RecvBytes(0)
		if err != nil {
			return err
		}

		log.Debug().Msg("Received imaging data")

		// Parse as protobuf message
		sensorOutput := &pb_outputs.SensorOutput{}
		err = proto.Unmarshal(sensorBytes, sensorOutput)
		if err != nil {
			log.Err(err).Msg("Failed to unmarshal trajectory data")
			continue
		}

		// Parse imaging data
		imagingData := sensorOutput.GetCameraOutput()
		if imagingData == nil {
			log.Warn().Msg("Received sensor data that was not camera data")
			continue
		}

		// Get trajectory
		trajectory := imagingData.GetTrajectory()
		if trajectory == nil {
			log.Warn().Msg("Received sensor data that was not trajectory data")
			continue
		}

		// Get the first trajectory point
		trajectoryPoints := trajectory.GetPoints()
		if len(trajectoryPoints) == 0 {
			log.Warn().Msg("Received sensor data that had no trajectory points")
			continue
		}
		firstPoint := trajectoryPoints[0]
		// This is the middle of the longest consecutive slice, it should be in the middle of the image (horizontally)

		// Use the PID controller to decide where to go
		//fmt.Println("Error: ", uint32(desiredTrajectoryPoint) - firstPoint.X)
		fmt.Println("Error: ", desiredTrajectoryPoint, " and ", firstPoint.X)
		pidController.Update(pid.ControllerInput{
			ReferenceSignal:  float64(desiredTrajectoryPoint),
			ActualSignal:     float64(firstPoint.X),
			SamplingInterval: 100 * time.Millisecond,
		})
		steerValue := pidController.State.ControlSignal
		log.Info().Float64("steerValue", steerValue).Int("Desired", desiredTrajectoryPoint).Float32("Actual", float32(firstPoint.X)).Msg("Calculated steering value")
		log.Info().Float32("", speed).Msg("Current Speed")
		log.Info().Float32("", kp).Msg("Current KP")
		log.Info().Float32("", kd).Msg("Current KD")
		// min-max
		if steerValue > 1 {
			steerValue = 1
		} else if steerValue < -1 {
			steerValue = -1
		}
		// todo! remove, actuator buggy
		steerValue = -steerValue

		// Create controller output, wrapped in generic sensor output
		controllerOutput := &pb_outputs.SensorOutput{
			SensorId:  1,
			Timestamp: uint64(time.Now().UnixMilli()),
			SensorOutput: &pb_outputs.SensorOutput_ControllerOutput{
				ControllerOutput: &pb_outputs.ControllerOutput{
					SteeringAngle: float32(steerValue),
					LeftThrottle:  speed,
					RightThrottle: speed,
					FrontLights:   false,
				},
			},
		}

		// Marshal the controller output
		controllerBytes, err := proto.Marshal(controllerOutput)
		if err != nil {
			log.Err(err).Msg("Failed to marshal controller output")
			continue
		}

		// Send it for the actuator (and others) to use
		_, err = outputSock.SendBytes(controllerBytes, 0)
		if err != nil {
			log.Err(err).Msg("Failed to send controller output")
			continue
		}

		log.Debug().Msg("Sent controller output")
	}
}

func onTuningState(newtuning *pb_systemmanager_messages.TuningState) {
	log.Info().Str("Value", newtuning.String()).Msg("Received tuning state from system manager")
	// shareddata.Tuning = newtuning
	// pidController.Reset()
}

func main() {
	servicerunner.Run(run, onTuningState, false)
}
