package platform

// StartAudioCapture begins microphone capture. Audio is buffered in memory.
func (g *glfwPlatform) StartAudioCapture() error { return g.audioEngine.startCapture() }

// IsAudioCapturing returns true if microphone capture is active.
func (g *glfwPlatform) IsAudioCapturing() bool {
    g.audioEngine.mu.Lock()
    v := g.audioEngine.capturing
    g.audioEngine.mu.Unlock()
    return v
}

// PollAudioCapture dequeues any available captured audio into the in-memory buffer.
func (g *glfwPlatform) PollAudioCapture() { g.audioEngine.pollCapture() }

// StopAudioCapture stops capture and returns a WAV byte slice (PCM 16-bit mono) in memory.
func (g *glfwPlatform) StopAudioCapture() ([]byte, error) { return g.audioEngine.stopCapture() }

