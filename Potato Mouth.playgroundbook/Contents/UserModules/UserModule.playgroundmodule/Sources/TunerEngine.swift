//
//  Microphone.swift
//  Tuner
//
//  Created by Erik Nordlund on 3/24/20.
//  Copyright © 2020 Erik Nordlund. All rights reserved.
//

import Foundation
import Accelerate
import AVFoundation
//import Dispatch
import SwiftUI


func signalGenerator(noiseAmount: Float, numSamples: Int, sampleRate: Int) -> [Float] {
//	let tau = Float.pi * 2
//
//    return (0 ..< numSamples).map { i in
//        let phase = Float(i) / Float(numSamples) * tau
//
//        var signal = cos(phase * 1) * 1.0
//        signal += cos(phase * 2) * 0.8
//        signal += cos(phase * 4) * 0.4
//        signal += cos(phase * 8) * 0.8
//        signal += cos(phase * 16) * 1.0
//        signal += cos(phase * 32) * 0.8
//
//        return signal + .random(in: -1...1) * noiseAmount
//    }
	let frequencies: [Float] = [1000, 2000, 3000]

	let tau: Float = .pi * 2

	return (0 ..< numSamples).map { index in
		frequencies.reduce(0) { accumulator, frequency in
			let normalizedIndex = Float(index) / Float(sampleRate)// scaling frequency to sample rate
			return accumulator + sin(normalizedIndex * frequency * tau) + .random(in: -1...1) * noiseAmount
		}
	}
}



struct TunerEngineUIState {
	let frequency: Float
	let noteLabelIndex: Int
	let cents: Float
//	var wormRotationDelta: Int// should only be -1, 0, or 1.
	let centerAngleOffset: Float
	var startAngle: Float
	var endAngle: Float
	var updating: Bool
//	init(frequency: Float, noteLabelIndex: Int, cents: Float, centerAngleOffset: Float, startAngle: Float, endAngle: Float) {
//		self.frequency = frequency
//		self.noteLabelIndex = noteLabelIndex
//		self.cents = cents
//		self.centerAngleOffset = centerAngleOffset
//		self.startAngle = startAngle
//		self.endAngle = endAngle
//	}
	
}

final public class TunerEngine: NSObject, AVCaptureAudioDataOutputSampleBufferDelegate, ObservableObject {
	@Published var enabled: Bool = false
	@Published var isActive = false
	@Published var uiState: TunerEngineUIState?
	
	@Published var inputSensitivity: Float = -7600000
	
	var maxCents: Float
	var tolerance: Float
	
	
//	@Published var fundamentalFrequency: Float?
//	@Published var fundamentalFrequencyPair: (Int, Float)?
	
//	private var enabled: Bool = false
	
//	private var n: vDSP_Length
	private var signal: [Float]
	
	
	
	private var forwardInputReal: [Float]
	private var forwardInputImag: [Float]
	private var forwardOutputReal: [Float]
	private var forwardOutputImag: [Float]
	
	
	private var fftSetUp: vDSP.FFT<DSPSplitComplex>
	
	private let sampleRate = 44100
	
	
	#warning("Find a way to reduce the buffer size for performance (response and computation)")
	private let bufferSize = vDSP_Length(32768)//vDSP_Length(1024)
	
	private var hertzScalar: Float
	
	private var lastSemitones: Float = 0.0
	var lastCenterAngleOffset: Float = 0.0
	
	
	// AVCaptureAudioDataOutputSampleBufferDelegate
	private var avOutput: AVCaptureAudioDataOutput
	private let audioDataQueue = dispatch_queue_serial_t(label: "AudioDataOutputQueue")
	
	private let captureSession = AVCaptureSession()
	
	public init(maxCents: Float, tolerance: Float) {
		self.maxCents = maxCents
		self.tolerance = tolerance
		
//		let n = vDSP_Length(2048)
		hertzScalar = Float(sampleRate) / Float(bufferSize)

		signal = signalGenerator(noiseAmount: 0.1, numSamples: Int(bufferSize), sampleRate: sampleRate)
		



		let log2n = vDSP_Length(log2(Float(bufferSize)))

		guard let fftSetUpInit = vDSP.FFT(log2n: log2n,
									  radix: .radix2,
									  ofType: DSPSplitComplex.self) else {
										fatalError("Can't create FFT Setup.")
		}
		fftSetUp = fftSetUpInit
		
		
		let halfN = Int(bufferSize / 2)
		
		forwardInputReal = [Float](repeating: 0,
									   count: halfN)
		forwardInputImag = [Float](repeating: 0,
									   count: halfN)
		forwardOutputReal = [Float](repeating: 0,
										count: halfN)
		forwardOutputImag = [Float](repeating: 0,
										count: halfN)
		
		
		avOutput = AVCaptureAudioDataOutput()
		circularBuffer = [Float](repeating: 0.0, count: Int(bufferSize))
		super.init()
		avOutput.setSampleBufferDelegate(self, queue: audioDataQueue)
		
	}
	
	var circularBuffer: [Float]
	var circularBufferIndex = 0
	
	func appendToCircularBuffer(value: Float) {
//		print("append(), \(circularBufferIndex)")
		if circularBufferIndex > (Int(bufferSize) - 1) {
			circularBufferIndex = 1
			circularBuffer[0] = value
		} else {
			circularBuffer[circularBufferIndex] = value
			circularBufferIndex += 1
		}
	}
	
	
	
	// AVCaptureAudioDataOutputSampleBufferDelegate
	public func captureOutput(_ output: AVCaptureOutput, didOutput sampleBuffer: CMSampleBuffer, from connection: AVCaptureConnection) {
		var dataBuffer = CMSampleBufferGetDataBuffer(sampleBuffer)
		
		let sampleCount = CMSampleBufferGetNumSamples(sampleBuffer)
		
		var bufferList = AudioBufferList()
		
		CMSampleBufferGetAudioBufferListWithRetainedBlockBuffer(sampleBuffer, bufferListSizeNeededOut: nil, bufferListOut: &bufferList, bufferListSize: MemoryLayout.size(ofValue: bufferList), blockBufferAllocator: nil, blockBufferMemoryAllocator: nil, flags: 0, blockBufferOut: &dataBuffer)
		
		let samples = UnsafeMutableBufferPointer<Int16>(start: UnsafeMutablePointer(OpaquePointer(bufferList.mBuffers.mData)), count: Int(bufferList.mBuffers.mDataByteSize)/MemoryLayout<Int16>.size)
		
		for sample in samples {
			appendToCircularBuffer(value: Float(sample))
		}
		
		// See "Capture Freeze" note in Noto for explanation (addresses unbalanced retain by CMSampleBufferGetAudio...)
		if dataBuffer != nil {
			Unmanaged.passUnretained(dataBuffer!).takeRetainedValue()
		}
		
		process(buffer: circularBuffer)
	}
	
	
	private func attemptCaptureSetup() {
		switch AVCaptureDevice.authorizationStatus(for: .audio) {
		case .authorized:
			setupCaptureSession()
			
		case .notDetermined:
			AVCaptureDevice.requestAccess(for: .audio) { [weak self] wasGranted in
				if wasGranted {
					self?.setupCaptureSession()
				}
			}
			
		case .denied:
			return
			
		case .restricted:
			return
		@unknown default:
			fatalError("Unknown audio permissions")
		}
	}
	
	private func setupCaptureSession() {
		captureSession.beginConfiguration()
		
		guard let audioDevice = AVCaptureDevice.default(for: .audio) else { return }
		
		do {
			let audioInput = try AVCaptureDeviceInput(device: audioDevice)
			
			if captureSession.canAddInput(audioInput) {
				captureSession.addInput(audioInput)
			}
		} catch {
			print("CAPTURE SESSION ERROR")
			return
		}
		
		
		
		if captureSession.canAddOutput(avOutput) {
			captureSession.addOutput(avOutput)
			
			captureSession.commitConfiguration()
			
			captureSession.startRunning()
		} else if captureSession.outputs.contains(avOutput) {
			
			captureSession.commitConfiguration()
			captureSession.startRunning()
		} else {
			captureSession.commitConfiguration()
		}
	}
	
	
	
	// testing purposes
	func updateSignal() {
		signal = signalGenerator(noiseAmount: 0.1, numSamples: Int(bufferSize), sampleRate: sampleRate)
//		process(buffer: signal)
	}
	
	
	
	//
	// Returns unfiltered component frequencies
	//
	func fft(buffer: [Float]) -> [Float] {
		forwardInputReal.withUnsafeMutableBufferPointer { forwardInputRealPtr in
			forwardInputImag.withUnsafeMutableBufferPointer { forwardInputImagPtr in
				forwardOutputReal.withUnsafeMutableBufferPointer { forwardOutputRealPtr in
					forwardOutputImag.withUnsafeMutableBufferPointer { [weak self] forwardOutputImagPtr in
						
						// creating a 'DSPSplitComplex' to contain the signal
						var forwardInput = DSPSplitComplex(realp: forwardInputRealPtr.baseAddress!, imagp: forwardInputImagPtr.baseAddress!)
						
						// converting the real values in 'signal' to complex numbers
						buffer.withUnsafeBytes {
							vDSP.convert(interleavedComplexVector: [DSPComplex]($0.bindMemory(to: DSPComplex.self)), toSplitComplexVector: &forwardInput)
						}
						
						// creating a 'DSPSplitComplex' to receive the FFT result
						var forwardOutput = DSPSplitComplex(realp: forwardOutputRealPtr.baseAddress!, imagp: forwardOutputImagPtr.baseAddress!)
						
						// perform the forward FFT
						self?.fftSetUp.forward(input: forwardInput, output: &forwardOutput)
						
						
						
					}
				}
			}
		}
//		print(self.forwardOutputImag)
		return forwardOutputImag
	}
	
	
	func getFundamentalFrequencyPair(fromBuffer: [Float]) -> (Int, Float) {
		#warning("This needs to be better than using the loudest frequency.")
		let frequencyPair = fft(buffer: fromBuffer).enumerated().reduce((0, Float.greatestFiniteMagnitude)) {
			// look for minimum value?
			// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ update this to include a user-selected(?) sensitivity/threshold, like in the experiment above
			// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ ---- input level for the mic is probably a seperate setting
			($0.element < $1.element) ? $0 : $1
			
		}//.0 + 1//accessing offset of resulting tuple: (offset, element), where element is the resulting minimum magnitude
		// + 1, because the array is index 0
		
//		frequencyPair.0 += 1// stopped doing this, because it was adding (sampleRate / bufferSize) to the frequency
		
		return frequencyPair
	}
	
	
	
	
	
	
	
	
	
	
	
	let referenceA4: Float = 440.0
	
	
	func makeUIState(frequency: Float, updating: Bool) -> TunerEngineUIState {
		// caculate semitones
		let octaves = log2(frequency / referenceA4)
		let semitones = 12 * octaves
		
		// calculate cents and note label
		// get the +/- 50 cents value
		// first, get the fraction of a semitone
		// scale the fraction to make it useful for multiplying by 50 later
		var centsOffset = abs(semitones.truncatingRemainder(dividingBy: 1.0))
		// if the value is at least one, the original fraction was at least 0.5, which means that it should be represented as the higher semitone, but flat
		if centsOffset >= 0.5 {
			if semitones >= 0 {
				// positive
				centsOffset = -100.0 * (1 - centsOffset)
			} else {
				// negative
				centsOffset = 100.0 * (1 - centsOffset)
			}
		} else {
			if semitones >= 0 {
				// positive
				centsOffset = 100.0 * centsOffset
			} else {
				// negative
				centsOffset = -100.0 * centsOffset
			}
		}
		// rounding to 10's decimal place for UI
		centsOffset = roundf(centsOffset * 10.0) / 10.0
		
		
		
		// For 2.2 semitones, the note name would be B.
		// For 2.6 semitones, the note name would be C.
		// Basically, just round the semitones to nearest integer
		// this is inpired by the constant semitones, but scales it down to a single octave range by subtracting the octaves floor.
		var noteIndex = Int(roundf(12 * (octaves.truncatingRemainder(dividingBy: 1.0))))
		if noteIndex < 0 {
			// shift index to the correct note if the note is "lower than A"
			noteIndex += 12
		} else if noteIndex > 11 {
			noteIndex -= 12
		}
		
//		print(noteIndex)
		
		var newCenterAngleOffset = lastCenterAngleOffset
		if uiState == nil {
			newCenterAngleOffset = 0.0
		} else if roundf(semitones) < roundf(lastSemitones) {
			// start angle needs to rotate a little less than halfway first, for animation clarity
			
			newCenterAngleOffset -= 360.0
			
		} else if roundf(semitones) > roundf(lastSemitones) {
			// end angle needs to rotate a little less than halfway first, because it seems to jump the animation as is
			newCenterAngleOffset += 360.0
		}
		
//		lastCenterAngleOffset = newCenterAngleOffset
		lastSemitones = semitones
		
		
		
		
		
		return TunerEngineUIState(frequency: frequency,
								  noteLabelIndex: noteIndex,
								  cents: centsOffset,
								  centerAngleOffset: newCenterAngleOffset,
								  startAngle: startAngleFloat(fromCents: centsOffset, withCenterOffsetAngle: newCenterAngleOffset),
								  endAngle: endAngleFloat(fromCents: centsOffset, withCenterOffsetAngle: newCenterAngleOffset),
								  updating: updating)
		
	}
	
	
	func startAngleFloat(fromCents: Float, withCenterOffsetAngle: Float) -> Float {
        if fromCents >= 0 {
            if fromCents <= tolerance {
                let delta = max(90.0 * (fromCents / maxCents), 0.001)
                return -90.0 - delta + withCenterOffsetAngle
            } else {
                return -90.001 + withCenterOffsetAngle
            }
        } else {
            let delta = 90.0 * (fromCents / maxCents)
            return -90.0 + delta + withCenterOffsetAngle
        }
    }
    
	func endAngleFloat(fromCents: Float, withCenterOffsetAngle: Float) -> Float {
		
        if fromCents <= 0 {
            if fromCents >= (-1 * tolerance) {
                let delta = min(90.0 * (fromCents / maxCents), -0.001)
                return -90.0 - delta + withCenterOffsetAngle
            } else {
                return -89.999 + withCenterOffsetAngle
            }
        } else {
            let delta = 90.0 * (fromCents / maxCents)
            return -90.0 + delta + withCenterOffsetAngle
        }
    }
	
	
	
	
	func updateUIState(newState: TunerEngineUIState?) {
		uiState = newState
		
		if newState != nil {
			isActive = true
		} else {
			isActive = false
		}
		
//		print("isActive (update)", isActive)
	}
	
//	var pairSum: Float = 0
//	var pairCount: Int = 0
	
	
	func process(buffer: [Float]) {
		// executing in background to preserve live interaction/animation
		#warning("Might want to share the AudioDataQueue here instead (could ensure blocking of audio buffer while processing, reducing memory usage)")
//		DispatchQueue.global(qos: .userInitiated).async {
			// perform analysis of input signal buffer
			let pair = getFundamentalFrequencyPair(fromBuffer: buffer)
			
			// make sure it's above some threshold
			if pair.1 < inputSensitivity {
//				pairSum += pair.1
//				pairCount += 1
//				print(pairSum / Float(pairCount))
				// frequency is loud enough to display
				// convert to actual frequency
				
				var tempUIState = makeUIState(frequency: Float(pair.0) * hertzScalar, updating: false)
				
//					print("temp centerAngleOffset = \(tempUIState.centerAngleOffset)")
//					print("last centerAngleOffset = \(self.lastCenterAngleOffset)")
				
				// check if the worm needs to rotate
				if tempUIState.centerAngleOffset == lastCenterAngleOffset {
//					print("no change")
					// no change
					
//						sleep(2)
					
					DispatchQueue.main.async { [weak self] in
						// publishing value to main thread
						
//						self?.uiState = tempUIState
						self?.updateUIState(newState: tempUIState)
					}
					
//						Timer(timeInterval: 2.0, repeats: false, block: {_ in
//
//						})
					
				} else if tempUIState.centerAngleOffset > lastCenterAngleOffset {
//						print("center angle increased")
					// worm needs to rotate clockwise
					
					tempUIState.updating = true
					// after moving the end angle, complete the
					DispatchQueue.main.async { [weak self] in
						// publishing value to main thread
						
//						self?.uiState = tempUIState
						self?.updateUIState(newState: tempUIState)
					}
					
					
				} else {// centerAngleOffset < lastCenterAngleOffset
//						print("center angle decreased")
					// worm needs to rotate counterclockwise
					
					tempUIState.updating = true
					// after moving the end angle, complete the motion
					DispatchQueue.main.async { [weak self] in
						// publishing value to main thread
						
//						self?.uiState = tempUIState
						self?.updateUIState(newState: tempUIState)
					}
					
					
				}
				
				lastCenterAngleOffset = tempUIState.centerAngleOffset
				
			} else {
				DispatchQueue.main.async { [weak self] in
					// publishing value to main thread
					
//					self?.uiState = nil
					self?.updateUIState(newState: nil)
				}
			}
			
//		}
	}
	
	
	
	
	
	
	func start() {
		print("engine.start()")
		
		attemptCaptureSetup()
		
		#warning("enabled should probably be determined by whether setup is successful.")
		enabled = true
//		print("isActive: ", isActive)
	}
	
	
	func stop() {
		print("engine.stop()")
		captureSession.stopRunning()
		enabled = false
		
		#warning("This is a race condition with the final cycle of parallel processing that doesn't complete by this point, sometimes leaving isActive true. This shouldn't be an issue once activity is only determined by processing results, and start()/stop() go away.")
		updateUIState(newState: nil)
//		print("isActive: ", isActive)
	}
	
	
}





