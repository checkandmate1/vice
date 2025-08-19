// pkg/platform/glfw.go
// Copyright(c) 2022-2024 vice contributors, licensed under the GNU Public License, Version 3.
// SPDX: GPL-3.0-only

// This a slightly modified version of the GLFW/SDL2 infrastructure from
// imgui-go-examples, where the main addition is cursor handling
// (backported from imgui's backends/imgui_impl_glfw.cpp), and some
// additional handling of text input outside of the imgui path.

package platform

import (
	"fmt"
	gomath "math"
	"runtime"
	"strconv"
	"sync"

	"github.com/mmp/vice/log"
	"github.com/mmp/vice/math"

	"github.com/AllenDang/cimgui-go/imgui"
	"github.com/go-gl/gl/v2.1/gl"
	"github.com/go-gl/glfw/v3.3/glfw"
	"github.com/veandco/go-sdl2/sdl"
)

// glfwPlatform implements the Platform interface using GLFW.
type glfwPlatform struct {
	audioEngine

	imguiIO *imgui.IO

	window *glfw.Window
	config *Config

	time                   float64
	mouseJustPressed       [3]bool
	mouseCursors           [imgui.MouseCursorCOUNT]*glfw.Cursor
	currentCursor          *glfw.Cursor
	inputCharacters        string
	anyEvents              bool
	lastMouseX, lastMouseY float64
	multisample            bool
	windowTitle            string
	mouseCapture           math.Extent2D
	// These are the keys that are actively held down; for now just the
	// function keys, since all we currently need is F1 for beaconator.
	heldFKeys map[imgui.Key]interface{}

	mouseDeltaMode         bool
	mouseDeltaStartPos     [2]float32
	mouseDeltaWindowCenter [2]float32
	mouseDelta             [2]float32

	// audio capture state
	capturingAudio      bool
	captureAudioMutex   sync.Mutex
	capturePCMBuffer    []int16
	captureDeviceOpened bool
	captureDeviceID     sdl.AudioDeviceID
}

func putu16(b []byte, v uint16) { b[0] = byte(v); b[1] = byte(v >> 8) }
func putu32(b []byte, v uint32) { b[0] = byte(v); b[1] = byte(v >> 8); b[2] = byte(v >> 16); b[3] = byte(v >> 24) }