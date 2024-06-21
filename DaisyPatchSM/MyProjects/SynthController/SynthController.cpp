/**
 *
 * @author Jacob Gutierrez
 * @author Nik Gibson
 * @date 06/03/2024
 * @version 1001.0
 */

#include "daisy_patch_sm.h"
#include "daisysp.h"

using namespace daisy;
using namespace patch_sm;
using namespace daisysp;

/** @brief Initializes classes.
 *  @details Initializes the Daisy Patch Submodule, I2C communication, 
 *           LFO, ADSR envelope generators, buttons, LEDs.
 *
*/
DaisyPatchSM patch;
I2CHandle i2c;
Oscillator lfo[2];
Adsr env[2];
Switch GateButton, Toggles[2], Wave_Select[4];
SpiHandle spi_handle;
GPIO led, Chip_Select, SW_A, SW_B;


/** @brief The state of the envelope control. 
 * @details The enum values are used to switch between the two ADSR envelope generators.
*/
enum EnvelopeControlState {
    CONTROL_ENV1,
    CONTROL_ENV2
};
EnvelopeControlState EG_State = CONTROL_ENV1;

/** @brief The available waveforms. 
 * @details The enum values are used to select the waveform for the LFO.
*/
enum Waveforms {
    WAVE_SIN = 0,
    WAVE_TRI = 1,
    WAVE_SAW = 2,
    NOWAVE = 3,
    WAVE_SQUARE = 4
};

/** @brief The state of the LFO control. 
 * @details The enum values are used to switch between the two LFOs.
*/
enum LFOControlState {
    CONTROL_LFO1,
    CONTROL_LFO2
};
LFOControlState lfoState = CONTROL_LFO1;

// I2C Variables
#define addr 0x60
#define BUFF_SIZE 8
#define SAMPLE_RATE 48000
static uint8_t DMA_BUFFER_MEM_SECTION output_buffer[BUFF_SIZE];
static unsigned int keyMapBk[8][5] = {0};
static unsigned int keyMapMk[8][5] = {0};
static unsigned int prevScan[8][5] = {0};
static unsigned int keyEvent[8][5] = {0};
static int keyPressTime[8][5] = {0};
static int octave;
static float lastVoltage;

// Global Variables
uint16_t EnvelopeGenerator[2], LFO[2], Velocity, Key;
float lfo_sig[2];
int waveform, previousWave;
bool env_state, GATE;

/** Timer variable to track duration when all wave selects are low */
uint32_t low_start_time = 0;
const uint32_t delay_threshold = 25; // 25 ms threshold

void readRowInput(unsigned char t, uint8_t* reg_1, uint8_t* reg_2){// unsigned char *br, unsigned char *mk) {
	//Load inputs
	uint8_t buf = (1 << static_cast<unsigned int>(t));
	spi_handle.BlockingTransmit(&buf,1);

  	// latch inputs into shift register
	Chip_Select.Write(1); //High Means No Load, only Serial Shift

  	// read first input shift register
	spi_handle.BlockingReceive(reg_1,1,100);

	// read second shift register and write next output shift register
	spi_handle.BlockingReceive(reg_2,1,100);

  	// start loading action again
  	Chip_Select.Write(0);
}

void ScanMatrix(void){
		uint8_t mkBuffer[1] = {0};
		uint8_t bkBuffer[1] = {0};
		uint8_t bit_value = 0;


		for(unsigned char k=0;k<8;k++){	//Iterates for T0-T7 //This is the actual scan
		readRowInput(k, bkBuffer, mkBuffer);	//Reads all 16bits (10 used) output of a single input

			for(unsigned char j=0;j<5;j++){	//Splices Data For Each "F-OCT"

			//BKX
			bit_value = 1&&(bkBuffer[0]&(1<<j));
			if(bit_value != (1&&keyMapBk[k][j])){
				keyMapBk[k][j] = bit_value;

			} 

			//MKX
			bit_value = 1&&(mkBuffer[0]&(1<<j));
			if(bit_value != (1&&keyMapMk[k][j])){
				keyMapMk[k][j] = bit_value;
			} 
		}
		}
}

void initMatrix(void){
	//Pin Initalization
	Chip_Select.Init(patch.D7, GPIO::Mode::OUTPUT);
	SW_A.Init(patch.D6, GPIO::Mode::INPUT, GPIO::Pull::PULLDOWN);	//MAKE AND TEST WITH PULL DOWNS
	SW_B.Init(patch.D5, GPIO::Mode::INPUT, GPIO::Pull::PULLDOWN);

    

	SpiHandle::Config spi_conf;

	//Set Configurations
	spi_conf.periph = SpiHandle::Config::Peripheral::SPI_2;				//IDK Which SPI but they used 2
	spi_conf.mode = SpiHandle::Config::Mode::MASTER;					//I am the sensei
	spi_conf.direction = SpiHandle::Config::Direction::TWO_LINES;		//Both Lines in Full Duplex
	spi_conf.nss = SpiHandle::Config::NSS::HARD_OUTPUT;					//NSS in Use - Daisy is Leader
	spi_conf.baud_prescaler = SpiHandle::Config::BaudPrescaler::PS_16; 	// Sets Baud to 25MHz/PS_X 
	spi_conf.pin_config.sclk = DaisyPatchSM::D10;						//SCLK Pin D10
	spi_conf.pin_config.miso = DaisyPatchSM::D8;						//MISO Pin D8
	spi_conf.pin_config.mosi = DaisyPatchSM::D9;						//MOSI Pin D9
	spi_conf.pin_config.nss = Pin();							        //CS Pin D1
	spi_conf.clock_phase = SpiHandle::Config::ClockPhase::ONE_EDGE;		//Still dont know which of these are right
	spi_conf.datasize = 8; 												//Set to 8Bit

	spi_handle.Init(spi_conf);							                // Initialize the handle using our configuration
}

//Velocity Helper Function
void postVelocity(unsigned char k, unsigned char j){
	int currentTime = System::GetNow();
	int velocity = 8 * (currentTime - keyPressTime[k][j]);
	if(velocity > 500){	//Limit for Vout
		velocity = 500;
		}
	float out_min = 0.01;
    float out_max = 5.0;
    int in_min = 1;
    int in_max = 500;

    float mappedFloat = 5.0 - static_cast<float>(velocity - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
	Velocity = static_cast<uint16_t>((mappedFloat / 5.0f) * 4095.0f);
}

void keyProcessor(void){
	//Octave Control
	if(1 == SW_A.Read()){
		octave = 0;						//Down Octave
	} else if(1 == SW_B.Read()){
		octave = 2;						//Up Octave
	} else {
		octave = 1;						//Standard Octave
	}

	unsigned char hold = 0;				//Sets lowest key to key hit
	for(unsigned char j=0;j<5;j++){
		for(unsigned char k=0;k<8;k++){
			if(keyMapBk[k][j] != prevScan[k][j]){
				keyEvent[k][j] = 1;
				if(keyMapBk[k][j] == 1){
					keyPressTime[k][j] = System::GetNow();
				}
			} else {
				keyEvent[k][j] = 0;
			}
			if((keyMapBk[k][j]==1)){
				prevScan[k][j] = 1;
			} else {
				prevScan[k][j] = 0;
			}
			if((keyMapMk[k][j]==1)&&!hold){
				hold = 1;
                // Raise gate here
                GATE = true;

				uint8_t key = (j * 8) + k + 1;
				float voltage = (0.085 * key) + (1.02 * octave);

				if(voltage != lastVoltage){
					lastVoltage = voltage;

					Key = static_cast<uint16_t>((voltage / 5.0f) * 4095.0f);

                    if(env[0].IsRunning() || env[1].IsRunning()){
                        env[0].Retrigger(true);
                        env[1].Retrigger(true);
                    }

					//Velocity
					postVelocity(k,j);

					keyPressTime[k][j] = 0;}
				}}}
    if(!hold){
        // Lower Gate
        GATE = false;
    }
}



/**
 * Initializes the I2C communication for the device.
 *
 * This function configures the I2C peripheral with the specified settings and initializes it for master mode.
 * The I2C peripheral is configured to use I2C_1 as the peripheral, 400kHz speed, and the SCL and SDA pins are set to B7 and B8 respectively.
 *
 * @throws None
 */
void InitI2C() {
    I2CHandle::Config i2c_config;
    i2c_config.periph = I2CHandle::Config::Peripheral::I2C_1;
    i2c_config.speed = I2CHandle::Config::Speed::I2C_400KHZ;
    i2c_config.mode = I2CHandle::Config::Mode::I2C_MASTER;
    i2c_config.pin_config.scl = DaisyPatchSM::B7;
    i2c_config.pin_config.sda = DaisyPatchSM::B8;
    i2c.Init(i2c_config);
}

/**
 * Transmits the given DAC value using I2C communication.
 *
 * @param value The 16-bit value to be transmitted.
 *
 * @throws None
 */
void TransmitDACValue(uint16_t EG1, uint16_t EG2, uint16_t LFO2, uint16_t Velocity) {
    output_buffer[0] = static_cast<uint8_t>(EG1 >> 8);
    output_buffer[1] = static_cast<uint8_t>(EG1 & 0xFF);
    output_buffer[2] = static_cast<uint8_t>(EG2 >> 8);
    output_buffer[3] = static_cast<uint8_t>(EG2 & 0xFF);
    output_buffer[4] = static_cast<uint8_t>(LFO2 >> 8);
    output_buffer[5] = static_cast<uint8_t>(LFO2 & 0xFF);
    output_buffer[6] = static_cast<uint8_t>(Velocity >> 8);
    output_buffer[7] = static_cast<uint8_t>(Velocity & 0xFF);
    i2c.TransmitDma(addr, output_buffer, BUFF_SIZE, NULL, NULL);
}

/**
 * Updates the waveform state based on the state of the Wave_Select buttons.
 *
 * @return void
 *
 * @throws None
 */
void Wave_STM()
{
    bool all_low = true;
    previousWave = waveform;
    for (int i = 0; i < 3; i++)
    {
        if (Wave_Select[i].Pressed())
        {
            all_low = false;
            break;
        }
    }

    if (Wave_Select[0].Pressed())
    {
        waveform = Oscillator::WAVE_TRI;
        low_start_time = 0; // Reset the timer
    }
    else if (Wave_Select[1].Pressed())
    {
        waveform = Oscillator::WAVE_SAW;
        patch.SetLed(waveform == Oscillator::WAVE_SAW);
        low_start_time = 0; // Reset the timer
    }
    else if (Wave_Select[2].Pressed())
    {
        waveform = Oscillator::WAVE_SQUARE;
        low_start_time = 0; // Reset the timer
    }
    else if (all_low)
    {
        if (low_start_time == 0)
        {
            low_start_time = System::GetNow(); // Start the timer
        }
        uint32_t elapsed_time = System::GetNow() - low_start_time;
        if (elapsed_time >= delay_threshold)
        {
            waveform = Oscillator::WAVE_SIN;
        }
        else
        {
            waveform = previousWave;
        }
    }
    else
    {
        low_start_time = 0; // Reset the timer if not all low
    }

}

/**
 * Updates the LFO (Low Frequency Oscillator) with the current waveform and frequency.
 * @param frequency The frequency of the LFO.
 * @return void
 *
 * @throws None
 */
void UpdateLFOParameters(float frequency) {
    // Update wave state
    Wave_STM();

    // Set the LFO parameters based on the current state
    switch(lfoState){
        case CONTROL_LFO1:
            if(previousWave != waveform)
                lfo[0].Reset();

            lfo[0].SetWaveform(static_cast<uint8_t>(waveform));
            lfo[0].SetFreq(frequency);
            break;

        case CONTROL_LFO2:
            if(previousWave != waveform)
                lfo[1].Reset();

            lfo[1].SetWaveform(static_cast<uint8_t>(waveform));
            lfo[1].SetFreq(frequency);
            break;
    }
}

/**
 * Sets the Attack, Decay, Sustain, and Release parameters of an ADSR (Attack, Decay, Sustain, Release) envelope generator.
 *
 * @param env The ADSR envelope generator to set parameters for.
 * @param attack The attack time parameter.
 * @param decay The decay time parameter.
 * @param sustain The sustain level parameter.
 * @param release The release time parameter.
 *
 * @return None
 *
 * @throws None
 */
void SetADSRParameters(Adsr &env, float attack, float decay, float sustain, float release) {
    env.SetAttackTime(attack);
    env.SetDecayTime(decay);
    env.SetSustainLevel(sustain);
    env.SetReleaseTime(release);
}

/**
 * Updates the ADSR parameters based on the current state.
 *
 * @param attack the attack time parameter
 * @param decay the decay time parameter
 * @param sustain the sustain level parameter
 * @param release the release time parameter
 *
 * @throws None
 */
void UpdateADSRParameters(float attack, float decay, float sustain, float release) {
    switch (EG_State) {
        case CONTROL_ENV1:
            SetADSRParameters(env[0], attack, decay, sustain, release);
            break;
        case CONTROL_ENV2:
            SetADSRParameters(env[1], attack, decay, sustain, release);
            break;
    }
}

/**
 * Debounces the buttons for stable input.
 *
 * @param None
 *
 * @return None
 *
 * @throws None
 */
void DebounceButtons() {
    GateButton.Debounce();
    for(int i = 0; i < 3; i++) {
        Wave_Select[i].Debounce();

        if(i < 2)
            Toggles[i].Debounce();
    }
}

/**
 * Callback function for the envelope.
 *
 * @param output Pointer to a pointer to an array of uint16_t values representing the output.
 * @param size The size of the output array.
 *
 * @throws None
 */
void OLACallback(uint16_t **output, size_t size) {
    
    for (size_t i = 0; i < size; i++) {
        // Process LFOs and ADSRs
        for (int j = 0; j < 2; j++) {
            lfo_sig[j] = lfo[j].Process() + 1.0f * 0.5f;
            LFO[j] = static_cast<uint16_t>(lfo_sig[j] * 4095.0f);
            EnvelopeGenerator[j] = static_cast<uint16_t>(env[j].Process(GATE) * 4095.0f);
        }

        // ADD 1V/Oct to out 0 when integration with Nik is complete
        output[0][i] = Key; //CV_OUT_1
        output[1][i] = LFO[0];  //CV_OUT_2
    }
}

int main(void) {
    patch.Init();
    InitI2C();

    // Initialize the LFOs, ADSRs and switches
    Pin Toggle_pins[2] = {patch.A9, patch.D1};
    Pin Wave_pins[3] = {patch.D2, patch.D3, patch.D4};
    float ADSR_knobs[4];
    float LFO_knob, mapped_lfo_freq;
    waveform = Waveforms::WAVE_SIN;
    GATE = false;

    // Initialize switches and buttons
    for(int j = 0; j < 3; j++) {
        Wave_Select[j].Init(Wave_pins[j], Switch::TYPE_TOGGLE);

        if(j < 2){
            lfo[j].Init(SAMPLE_RATE);
            env[j].Init(SAMPLE_RATE);
            Toggles[j].Init(Toggle_pins[j], Switch::TYPE_TOGGLE);

            SetADSRParameters(env[j], 0.01f, 0.1f, 0.8f, 0.1f);
            lfo[j].SetFreq(1.0f);
        }
    }
    
	initMatrix();

    // Start DAC Callback function
    patch.StartDac(OLACallback);

    // Loop forever
    while (1) {
        patch.ProcessAnalogControls();
        DebounceButtons();
		
    
        // Get Envelope knob values
        for(int i = 0; i < 4; i++) {
            ADSR_knobs[i] = patch.GetAdcValue(i);
        }

        env_state = GateButton.Pressed();

        // Get knob value for LFO and map to [0.5, 50] Hz
        LFO_knob = patch.GetAdcValue(CV_5);
        mapped_lfo_freq  = fmap(LFO_knob, 0.5f, 50.0f);

        // Update the state based on the toggle switch
        EG_State = Toggles[0].Pressed() ? CONTROL_ENV2 : CONTROL_ENV1;

        // Update LFO based on toggle switch
        lfoState = Toggles[1].Pressed() ? CONTROL_LFO2 : CONTROL_LFO1;

        UpdateADSRParameters(ADSR_knobs[0], ADSR_knobs[1], ADSR_knobs[2], ADSR_knobs[3]);

        UpdateLFOParameters(mapped_lfo_freq);

		ScanMatrix();
		keyProcessor();

        TransmitDACValue(EnvelopeGenerator[0], EnvelopeGenerator[1], LFO[1], Velocity);
    }
}
