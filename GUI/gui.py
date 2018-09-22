from communication import Sensor
import matplotlib.pyplot as plt
from time import sleep
from kivy.app import App
from kivy.clock import Clock
from kivy.config import Config
from kivy.garden.matplotlib.backend_kivyagg import FigureCanvasKivyAgg
from kivy.lang import Builder
from kivy.uix.accordion import AccordionItem
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.label import Label
from kivy.uix.popup import Popup
from kivy.uix.screenmanager import Screen, ScreenManager
from kivy.uix.textinput import TextInput
import numpy as np

Config.set('graphics', 'width', '800')
Config.set('graphics', 'height', '800')

text_color = [0.78, 0.78, 0.78]
background_color = [0.12, 0.12, 0.12]
error_text_color = [0.73, 0.25, 0.27]
axes_bg_color = [0.34, 0.34, 0.34]
axes_line_color = [0.2, 0.64, 0.81]

sensor = Sensor()

def convert_raw_to_g(raw_value):
    offset = 0
    factor = 0.0244140625
    real_val = (raw_value+offset)*factor
    return real_val

class SensorScreenManager(ScreenManager):
    pass

class ConnectScreen(Screen):

    def update_padding(self, text_input, *args):
        text_width = text_input._get_text_width(
            text_input.text,
            text_input.tab_width,
            text_input._label_cached
        )

        if (text_width != 0):
            text_input.padding_x = (text_input.width - text_width)/2
        else:
            text_input.padding_x = (123, 123)

    def connect_to_sensor(self, text_input):
        if (sensor.connect(address = text_input)):
            self.ids.label_error.text = ''
            screen_manager.current = 'screen_control'
        else:
            self.ids.label_error.text = 'Could not connect. Try making a sensor reset.'

class ControlScreen(Screen):

    def __init__(self, **kwargs):
        super(ControlScreen, self).__init__(**kwargs)
        self.time_elapsed_from_last_threshold_exceeded = 0
        self.time_signal = np.array([])
        self.time_signal_x = np.array([])
        self.fft_signal = np.array([])
        self.fft_signal_x = np.array([])

    def measurement_control_progressbar_callback(self, dt):
        if(self.ids.progressbar_measurement.value != 100):
            self.ids.progressbar_measurement.value += 1
        else:
            Clock.unschedule(self.measurement_control_progressbar_callback)
            self.popup = Popup(title='Information',
                          size_hint=(None, None), size=(400, 400))
            self.popup.open()
            self.popup.content = Label(
                text='Measurement finished. Downloading results...')
            Clock.unschedule(self.are_results_ready_callback)
            Clock.schedule_interval(self.are_results_ready_callback, 0.5)
            
    def are_results_ready_callback(self, dt):
        if(sensor.is_measurement_finished):
            Clock.unschedule(self.are_results_ready_callback)
            self.download_results()
            self.popup.dismiss()
            self.update_gui()
            self.ids.accordion_time_results.collapse = False
        else:
            pass

    def start_measurement(self, frequency, length):
        try:
            #little bug here if length<1, the dt is always about 0.01 but it looks better, becouse you are able
            #to see the progress
            sensor.trigger_measurement(frequency, length)
            self.time_signal_x = np.linspace(0.0, float(length), num=float(length)*float(frequency), endpoint=False)
            self.fft_signal_x = (float(frequency)/2)*np.linspace(0.0, 1.0, num=((float(length)*float(frequency))/2+1), endpoint=True)
            Clock.schedule_interval(
                self.measurement_control_progressbar_callback, float(length)/100)
            self.ids.progressbar_measurement.value = 0
        except:
            self.ids.progressbar_measurement.value = 0

    def download_results(self):
        self.rms = sensor.read_calculated_value("rms")
        self.average = sensor.read_calculated_value("average")
        self.max_val = sensor.read_calculated_value("max_val")
        self.min_val = sensor.read_calculated_value("min_val")
        self.amplitude = sensor.read_calculated_value("amplitude")
        self.crest_factor = sensor.read_calculated_value("crest_factor")
        self.time_signal = sensor.read_signal()
        self.fft_signal = sensor.read_fft()
        
    def update_gui(self):
        self.ids.accordion_time_results.update_figure(self.time_signal_x, self.time_signal)
        self.ids.accordion_fourier_results.update_figure(self.fft_signal_x, self.fft_signal)
        self.ids.textinput_indicator_rms.text = str((self.rms))
        self.ids.textinput_indicator_average.text = str((self.average))
        self.ids.textinput_indicator_maxval.text = str((self.max_val))
        self.ids.textinput_indicator_minval.text = str((self.min_val))
        self.ids.textinput_indicator_amplitude.text = str((self.amplitude))
        self.ids.textinput_indicator_crestfactor.text = str((self.crest_factor)) 

    def set_monitoring_threshold(self, threshold):
        sensor.set_threshold_for_threshold_exceeded_monitoring(threshold)
        Clock.unschedule(self.threshold_monitoring_callback)
        Clock.schedule_interval(self.threshold_monitoring_callback, 0.2)

    def threshold_monitoring_callback(self, dt):
        if(sensor.monitor_threshold_exceeded()):
            Clock.unschedule(self.threshold_monitoring_callback)
            self.handle_threshold_exceeded_event()
        else:
            pass

    def handle_threshold_exceeded_event(self):
        self.time_elapsed_from_last_threshold_exceeded = 0
        Clock.unschedule(
            self.update_threshold_exceeded_time_elapsed_label_callback)
        Clock.schedule_interval(
            self.update_threshold_exceeded_time_elapsed_label_callback, 0.2)

    def update_threshold_exceeded_time_elapsed_label_callback(self, dt):
        self.time_elapsed_from_last_threshold_exceeded += 0.2
        self.ids.label_time_from_last_threshold_exceeded.text = (
            'Time from last threshold exceeded event: {:.2f} s' .format(self.time_elapsed_from_last_threshold_exceeded))

class ResultsAccordion(AccordionItem):

    def create_figure(self):
        self.figure = plt.figure()
        self.figure.figsize = (1, 1)
        self.figure.patch.set_facecolor(background_color)
        self.ax = self.figure.add_subplot(1, 1, 1)
        self.ax.spines['right'].set_color(text_color)
        self.ax.spines['left'].set_color(text_color)
        self.ax.spines['top'].set_color(text_color)
        self.ax.spines['bottom'].set_color(text_color)
        self.ax.tick_params(axis='x', colors=text_color)
        self.ax.tick_params(axis='y', colors=text_color)
        self.ax.yaxis.label.set_color(text_color)
        self.ax.xaxis.label.set_color(text_color)
        self.ax.title.set_color(text_color)
        self.ax.patch.set_facecolor(axes_bg_color)
        self.ax.grid(True)

    def update_figure(self, vals_x, vals_y):
        self.ax.clear()
        self.ax.patch.set_facecolor(axes_bg_color)
        self.ax.grid(True)
        self.ax.plot(vals_x, vals_y)

class TimeResultsAccordion(ResultsAccordion):

    def __init__(self, **kwargs):
        super(TimeResultsAccordion, self).__init__(**kwargs)
        self.layout = BoxLayout(padding=[50, 0, 50, 150])
        self.create_figure()
        self.ax.set_title('Acceleration')
        self.ax.title.set_color(text_color)
        self.ax.set_xlabel('Time [s]')
        self.ax.set_ylabel('Acceleration [g]')
        self.layout.add_widget(FigureCanvasKivyAgg(self.figure))
        self.add_widget(self.layout)

    def update_figure(self, vals_x, vals_y):
        super(TimeResultsAccordion, self).update_figure(vals_x, vals_y)
        self.ax.set_title('Acceleration')
        self.ax.title.set_color(text_color)
        self.ax.set_xlabel('Time [s]')
        self.ax.set_ylabel('Acceleration [g]')

class FftResultsAccordion(ResultsAccordion):

    def __init__(self, **kwargs):
        super(FftResultsAccordion, self).__init__(**kwargs)
        self.layout = BoxLayout(padding=[50, 0, 50, 150])
        self.create_figure()
        self.ax.set_title('FFT')
        self.ax.title.set_color(text_color)
        self.ax.set_xlabel('Frequency [Hz]')
        self.ax.set_ylabel('Acceleration [g]')
        self.layout.add_widget(FigureCanvasKivyAgg(self.figure))
        self.add_widget(self.layout)

    def update_figure(self, vals_x, vals_y):
        super(FftResultsAccordion, self).update_figure(vals_x, vals_y)
        self.ax.set_title('FFT')
        self.ax.title.set_color(text_color)
        self.ax.set_xlabel('Frequency [Hz]')
        self.ax.set_ylabel('Acceleration [g]')

screen_manager = Builder.load_file("gui.kv")

class SensorApp(App):

    def build(self):
        return screen_manager

if __name__ == '__main__':
    SensorApp().run()
