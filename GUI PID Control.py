import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import time
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import threading
from queue import Queue

class PengendaliMotorDC:
    def __init__(self, root):
        self.root = root
        self.root.title("Pengendali Motor DC PID")
        self.root.geometry("1500x800")
        self.root.configure(bg="#B0C4DE")

        # Variabel
        self.is_terhubung = False
        self.arduino = None
        self.antrian_data = Queue()
        self.sedang_berjalan = False

        # Data Plot
        self.data_waktu = []
        self.data_rpm = []
        self.data_setpoint = []
        self.waktu_mulai = time.time()

        # Setup GUI
        self.setup_gui()

        # Setup Plot
        self.setup_plot()

        # Temukan port Arduino yang tersedia
        self.perbarui_port()

    def setup_gui(self):
        # Panel Kontrol
        frame_kontrol = ttk.LabelFrame(self.root, text="Panel Kontrol", padding="10")
        frame_kontrol.grid(row=0, column=0, padx=10, pady=5, sticky="nsew")
        frame_kontrol.configure(style="Custom.TLabelframe")

        ttk.Label(frame_kontrol, text="Port:").grid(row=0, column=0, padx=5, pady=5)
        self.var_port = tk.StringVar()
        self.combo_port = ttk.Combobox(frame_kontrol, textvariable=self.var_port)
        self.combo_port.grid(row=0, column=1, padx=5, pady=5)

        ttk.Button(frame_kontrol, text="Refresh", command=self.perbarui_port).grid(row=0, column=2, padx=5, pady=5)
        self.tombol_sambung = ttk.Button(frame_kontrol, text="Connect", command=self.toggle_sambungan)
        self.tombol_sambung.grid(row=0, column=3, padx=5, pady=5)

        ttk.Label(frame_kontrol, text="Kp:").grid(row=1, column=0, padx=5, pady=5)
        self.var_kp = tk.DoubleVar(value=1)
        self.entry_kp = ttk.Entry(frame_kontrol, textvariable=self.var_kp)
        self.entry_kp.grid(row=1, column=1, padx=5, pady=5)

        ttk.Label(frame_kontrol, text="Ki:").grid(row=2, column=0, padx=5, pady=5)
        self.var_ki = tk.DoubleVar(value=0.1)
        self.entry_ki = ttk.Entry(frame_kontrol, textvariable=self.var_ki)
        self.entry_ki.grid(row=2, column=1, padx=5, pady=5)

        ttk.Label(frame_kontrol, text="Kd:").grid(row=3, column=0, padx=5, pady=5)
        self.var_kd = tk.DoubleVar(value=0.05)
        self.entry_kd = ttk.Entry(frame_kontrol, textvariable=self.var_kd)
        self.entry_kd.grid(row=3, column=1, padx=5, pady=5)

        ttk.Button(frame_kontrol, text="Perbarui PID", command=self.perbarui_pid).grid(row=4, column=0, columnspan=3, pady=10)

        ttk.Label(frame_kontrol, text="Setpoint (RPM):").grid(row=5, column=0, padx=5, pady=5)
        self.var_setpoint = tk.IntVar(value=0)
        self.entry_setpoint = ttk.Entry(frame_kontrol, textvariable=self.var_setpoint)
        self.entry_setpoint.grid(row=5, column=1, padx=5, pady=5)

        # Tombol kontrol motor
        ttk.Button(frame_kontrol, text="Run", command=self.jalankan_motor_searah).grid(row=6, column=0, padx=1, pady=5)
        ttk.Button(frame_kontrol, text="Turn Around", command=self.jalankan_motor_berlawanan).grid(row=6, column=1, padx=1, pady=5)
        ttk.Button(frame_kontrol, text="Berhenti", command=self.berhentikan_motor).grid(row=6, column=2, padx=1, pady=5)

        self.label_rpm = ttk.Label(frame_kontrol, text="RPM Aktual: 0")
        self.label_rpm.grid(row=7, column=0, columnspan=3, pady=10)

        self.label_status = ttk.Label(frame_kontrol, text="Status: Tidak Terhubung")
        self.label_status.grid(row=8, column=0, columnspan=3, pady=10)

        self.style = ttk.Style()
        self.style.configure("Custom.TLabelframe", background="#B0C4DE")
        self.style.configure("Custom.TLabelframe.Label", background="#B0C4DE", foreground="black")
        self.style.configure("TLabel", background="#B0C4DE", foreground="black")

    def setup_plot(self):
        frame_plot = ttk.LabelFrame(self.root, text="Plot Grafik", padding="10")
        frame_plot.grid(row=0, column=1, padx=10, pady=5, sticky="nsew")
        frame_plot.configure(style="Custom.TLabelframe")

        self.fig = Figure(figsize=(10.2, 7.45), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.ax.set_title('Respon RPM Motor')
        self.ax.set_xlabel('Waktu (s)')
        self.ax.set_ylabel('RPM')
        self.ax.grid(True)

        self.canvas = FigureCanvasTkAgg(self.fig, master=frame_plot)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        self.garis_rpm, = self.ax.plot([], [], 'b-', label='RPM Aktual')
        self.garis_setpoint, = self.ax.plot([], [], 'r--', label='Setpoint')
        self.ax.legend(loc='upper left')

        self.ax.set_ylim(-1000, 1000)
        self.ax.set_xlim(0, 10)

    def perbarui_port(self):
        port = [p.device for p in serial.tools.list_ports.comports()]
        self.combo_port['values'] = port
        if port:
            self.combo_port.set(port[0])

    def toggle_sambungan(self):
        if not self.is_terhubung:
            try:
                port = self.var_port.get()
                self.arduino = serial.Serial(port, 115200, timeout=1)
                time.sleep(2)
                self.is_terhubung = True
                self.tombol_sambung.config(text="Putuskan")
                self.label_status.config(text="Status: Terhubung")

                self.sedang_berjalan = True
                threading.Thread(target=self.baca_serial, daemon=True).start()
                threading.Thread(target=self.perbarui_plot, daemon=True).start()

            except Exception as e:
                messagebox.showerror("Kesalahan Sambungan", str(e))
                self.label_status.config(text=f"Kesalahan: {str(e)}")
        else:
            self.sedang_berjalan = False
            if self.arduino:
                self.arduino.close()
            self.is_terhubung = False
            self.tombol_sambung.config(text="Sambungkan")
            self.label_status.config(text="Status: Tidak Terhubung")

    def perbarui_pid(self):
        if self.is_terhubung:
            kp = self.var_kp.get()
            ki = self.var_ki.get()
            kd = self.var_kd.get()
            
            # Kirim parameter PID ke Arduino
            self.arduino.write(f"P{kp}\n".encode('utf-8'))
            time.sleep(0.1)
            self.arduino.write(f"I{ki}\n".encode('utf-8'))
            time.sleep(0.1)
            self.arduino.write(f"D{kd}\n".encode('utf-8'))
            time.sleep(0.1)

    def jalankan_motor_searah(self):
        if self.is_terhubung:
            setpoint = self.var_setpoint.get()
            if 0 <= setpoint <= 10000:
                # Kirim perintah dengan awalan 'S' untuk searah jarum jam
                self.arduino.write(f"S{setpoint}\n".encode('utf-8'))
            else:
                messagebox.showerror("Kesalahan", "RPM harus antara 0 dan 10000")

    def jalankan_motor_berlawanan(self):
        if self.is_terhubung:
            setpoint = self.var_setpoint.get()
            if 0 <= setpoint <= 10000:
                # Kirim perintah dengan awalan 'B' untuk berlawanan jarum jam
                self.arduino.write(f"B{setpoint}\n".encode('utf-8'))
            else:
                messagebox.showerror("Kesalahan", "RPM harus antara 0 dan 10000")

    def berhentikan_motor(self):
        if self.is_terhubung:
            # Perintah 'X' untuk berhenti pada kode Arduino
            self.arduino.write(b"X\n")

    def baca_serial(self):
        while self.sedang_berjalan and self.is_terhubung:
            try:
                baris = self.arduino.readline().decode('utf-8').strip()
                # Cari baris dengan dua angka yang dipisahkan koma
                if ',' in baris:
                    try:
                        setpoint, rpm = map(float, baris.split(','))
                        waktu_saat_ini = time.time() - self.waktu_mulai
                        self.antrian_data.put((waktu_saat_ini, rpm, setpoint))
                    except ValueError:
                        # Lewati baris yang tidak dapat diurai
                        continue
            except Exception as e:
                self.label_status.config(text=f"Kesalahan: {str(e)}")
                break

    def perbarui_plot(self):
        while self.sedang_berjalan:
            try:
                while not self.antrian_data.empty():
                    waktu_saat_ini, rpm, setpoint = self.antrian_data.get()
                    self.data_waktu.append(waktu_saat_ini)
                    self.data_rpm.append(rpm)
                    self.data_setpoint.append(setpoint)

                    # Simpan hanya 100 titik data terakhir
                    if len(self.data_waktu) > 100:
                        self.data_waktu = self.data_waktu[-100:]
                        self.data_rpm = self.data_rpm[-100:]
                        self.data_setpoint = self.data_setpoint[-100:]

                    # Perbarui batas x-axis untuk menunjukkan 10 detik terakhir
                    self.ax.set_xlim(max(0, self.data_waktu[-1] - 10), self.data_waktu[-1])
                    self.garis_rpm.set_data(self.data_waktu, self.data_rpm)
                    self.garis_setpoint.set_data(self.data_waktu, self.data_setpoint)

                    self.canvas.draw()
                    self.label_rpm.config(text=f"RPM Aktual: {int(rpm)}")

                time.sleep(0.05)
            except Exception as e:
                self.label_status.config(text=f"Kesalahan Plot: {str(e)}")
                break

if __name__ == "__main__":
    root = tk.Tk()
    app = PengendaliMotorDC(root)
    root.mainloop()