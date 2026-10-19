import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
import tkinter as tk
from tkinter import ttk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import control as ctl
# حساب استجابة الخطوة للنظام
def calculate_step_response(num, den):
    system = ctl.TransferFunction(num, den)
    t, y = ctl.step_response(system)
    return t, y

# تحديث الرسم البياني للترانسفير فنكشن فقط
def update_transfer_function_plot():
    try:
        num = [float(i) for i in entry_num.get().split(',')]
        den = [float(i) for i in entry_den.get().split(',')]

        t, y = calculate_step_response(num, den)

        ax1.clear()
        ax1.plot(t, y, label="Transfer Function Response", color="green")
        ax1.set_title("Step Response for Transfer Function")
        ax1.set_xlabel("Time (s)")
        ax1.set_ylabel("Response")
        ax1.legend()
        ax1.grid(True)  # إضافة الشبكة للمحور الثاني
        canvas.draw()

    except Exception as e:
        label_error.config(text=f"Error: {e}")

# تطبيق التحكم PID
def apply_pid_control(num, den, Kp, Ki, Kd):
    G_p = ctl.TransferFunction(num, den)  # تمثيل النظام باستخدام دالة التحويل

    # تعريف وحدة التحكم PID
    # C(s) = Kp + Ki/s + Kd*s
    numerator_pid = [Kd, Kp, Ki]  # البسط: Kd*s^2 + Kp*s + Ki
    denominator_pid = [1, 0]      # المقام: s
    C_pid = ctl.TransferFunction(numerator_pid, denominator_pid)

    # النظام المغلق
    # T(s) = (C_pid(s) * G_p(s)) / (1 + C_pid(s) * G_p(s))
    numerator_closed = np.polymul(C_pid.num[0][0], G_p.num[0][0])  # البسط: C_pid(s) * G_p(s)
    denominator_closed = np.polyadd(np.polymul(C_pid.den[0][0], G_p.den[0][0]), np.polymul(C_pid.num[0][0], G_p.num[0][0]))  # المقام: 1 + C_pid(s) * G_p(s)

    # تعريف النظام المغلق
    T = ctl.TransferFunction(numerator_closed, denominator_closed)  # النظام المغلق

    t, y = ctl.step_response(T)
    return t, y

# تحديث الرسومات و القيم عند تغيير PID
def update_plots(*args):
    try:
        num = [float(i) for i in entry_num.get().split(',')]
        den = [float(i) for i in entry_den.get().split(',')]
        system = ctl.TransferFunction(num, den)
        t_before, y_before = ctl.step_response(system)
        step_info = ctl.step_info(system)  # استخدام step_info لاستخراج القيم مباشرة
        
        max_overshoot = step_info.get('Overshoot', 0)
        settling_time = step_info.get('SettlingTime', None)
        rise_time = step_info.get('RiseTime', None)
        peak_time = step_info.get('PeakTime', None)
        steady_state_error = abs(1 - y_before[-1]) * 100  # نسبة مئوية

        Kp = float(scale_Kp.get().strip())
        Ki = float(scale_Ki.get().strip())
        Kd = float(scale_Kd.get().strip())
        
        
        system_pid = ctl.TransferFunction(np.polymul([Kd, Kp, Ki], num), np.polyadd(np.polymul([1, 0], den), np.polymul([Kd, Kp, Ki], num)))
        step_info_pid = ctl.step_info(system_pid)  # استخدام step_info لاستخراج القيم مباشرة
        t_after, y_after = ctl.step_response(system_pid)
        max_overshoot1 = step_info_pid.get('Overshoot', 0)
        settling_time1 = step_info_pid.get('SettlingTime', None)
        rise_time1 = step_info_pid.get('RiseTime', None)
        peak_time1 = step_info_pid.get('PeakTime', None)
        steady_state_error1 = abs(1 - y_after[-1]) * 100  # نسبة مئوية

        ax1.clear()
        ax1.plot(t_before, y_before, label="Before PID Control", color="blue")
        ax1.set_title("Step Response Before Control")
        ax1.set_xlabel("Time (s)")
        ax1.set_ylabel("Response")
        ax1.legend()
        ax1.grid(True)
        ax1.grid(True)  # إضافة الشبكة للمحور الأول

        ax2.clear()
        ax2.plot(t_after, y_after, label="After PID Control", color="red")
        ax2.set_title("Step Response After Control")
        ax2.set_xlabel("Time (s)")
        ax2.set_ylabel("Response")
        ax2.legend()
        ax2.grid(True)  # إضافة الشبكة للمحور الثاني
        canvas.draw()

        # تحديث القيم
        label_max_overshoot_before.config(text=f"Max Overshoot Before: {max_overshoot:.2f}%")
        label_settling_time_before.config(text=f"Settling Time Before: {settling_time:.2f}s" if settling_time else "N/A")
        label_rise_time_before.config(text=f"Rise Time Before: {rise_time:.2f}s" if rise_time else "N/A")
        label_peak_time_before.config(text=f"Peak Time Before: {peak_time:.2f}s" if peak_time else "N/A")
        label_steady_state_error_before.config(text=f"Steady-State Error Before: {steady_state_error:.2f}%")

        label_max_overshoot_after.config(text=f"Max Overshoot After: {max_overshoot1:.2f}%")
        label_settling_time_after.config(text=f"Settling Time After: {settling_time1:.2f}s" if settling_time1 else "N/A")
        label_rise_time_after.config(text=f"Rise Time After: {rise_time1:.2f}s" if rise_time1 else "N/A")
        label_peak_time_after.config(text=f"Peak Time After: {peak_time1:.2f}s" if peak_time1 else "N/A")
        label_steady_state_error_after.config(text=f"Steady-State Error After: {steady_state_error1:.2f}%")

        # ربط القيم بطباعة النتائج
        button_print_values.config(command=lambda: print_calculated_values(
            max_overshoot, settling_time, rise_time, peak_time, steady_state_error,
            max_overshoot1, settling_time1, rise_time1, peak_time1, steady_state_error1
        ))
    except Exception as e:
        label_error.config(text=f"Error: {e}")

# طباعة القيم المحسوبة في نافذة جديدة
def print_calculated_values(max_overshoot_before, settling_time_before, rise_time_before, peak_time_before, steady_state_error_before,
                            max_overshoot_after, settling_time_after, rise_time_after, peak_time_after, steady_state_error_after):
    new_window = tk.Toplevel()
    new_window.title("Calculated Values")
    new_window.geometry("800x400")

    # إنشاء الجدول
    columns = ("Metric", "Before PID", "After PID", "Judgment")
    table = ttk.Treeview(new_window, columns=columns, show="headings", height=6)
    table.pack(fill=tk.BOTH, expand=True)

    # إعداد رؤوس الجدول
    for col in columns:
        table.heading(col, text=col)
        table.column(col, anchor=tk.CENTER)

    # إدخال البيانات
    def judge_overshoot(value):
        return "Good" if value < 10 else "Needs Improvement"

    def judge_settling_time(value):
        return "Good" if value and value < 5 else "Needs Improvement"

    def judge_steady_state_error(value):
        return "Good" if value < 2 else "Needs Improvement"

    def judge_rise_time(value):
        return "Good" if value and value < 3 else "Needs Improvement"

    def judge_peak_time(value):
        return "Good" if value and value < 2 else "Needs Improvement"

    table.insert("", tk.END, values=("Max Overshoot (%)", f"{max_overshoot_before:.2f}", f"{max_overshoot_after:.2f}", 
                                 f"{judge_overshoot(max_overshoot_after)}"))
    table.insert("", tk.END, values=("Settling Time (s)", 
                                 f"{settling_time_before:.2f}" if settling_time_before else "N/A", 
                                 f"{settling_time_after:.2f}" if settling_time_after else "N/A", 
                                 f"{judge_settling_time(settling_time_after)}"))
    table.insert("", tk.END, values=("Rise Time (s)", 
                                 f"{rise_time_before:.2f}" if rise_time_before else "N/A", 
                                 f"{rise_time_after:.2f}" if rise_time_after else "N/A", 
                                 f"{judge_rise_time(rise_time_after)}"))
    table.insert("", tk.END, values=("Peak Time (s)", 
                                 f"{peak_time_before:.2f}" if peak_time_before else "N/A", 
                                 f"{peak_time_after:.2f}" if peak_time_after else "N/A", 
                                 f"{judge_peak_time(peak_time_after)}"))
    table.insert("", tk.END, values=("Steady-State Error (%)", 
                                 f"{steady_state_error_before:.2f}", 
                                 f"{steady_state_error_after:.2f}", 
                                 f"{judge_steady_state_error(steady_state_error_after)}"))

    # زر إغلاق النافذة
    ttk.Button(new_window, text="Close", command=new_window.destroy).pack(pady=10)

# إنشاء واجهة المستخدم
root = tk.Tk()
root.title("PID Controller with Response Metrics")
root.geometry("1200x800")

# إعداد الرسم البياني
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(6, 8))
fig.tight_layout(pad=3.0)
ax1.grid(True)  # إضافة الشبكة للمحور الأول
ax2.grid(True)  # إضافة الشبكة للمحور الثاني
# تقسيم النافذة
frame_left = ttk.Frame(root, width=400, padding=10)
frame_left.pack(side=tk.LEFT, fill=tk.BOTH)

frame_right = ttk.Frame(root, width=800, padding=10)
frame_right.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

# إطار الرسومات
canvas = FigureCanvasTkAgg(fig, master=frame_right)
canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

# إدخال الترانسفير فنكشن
frame_inputs = ttk.LabelFrame(frame_left, text="Transfer Function Input", padding=10)
frame_inputs.pack(fill=tk.X, pady=5)

ttk.Label(frame_inputs, text="Numerator (comma separated):").grid(row=0, column=0, sticky=tk.W)
entry_num = ttk.Entry(frame_inputs, width=30)
entry_num.grid(row=0, column=1)

ttk.Label(frame_inputs, text="Denominator (comma separated):").grid(row=1, column=0, sticky=tk.W)
entry_den = ttk.Entry(frame_inputs, width=30)
entry_den.grid(row=1, column=1)

# زر لتحديث الرسم فقط للترانسفير فنكشن
button_update_transfer_function = ttk.Button(frame_left, text="Update Transfer Function Plot", command=update_transfer_function_plot)
button_update_transfer_function.pack(pady=10)

# التحكم بـ PID
frame_pid = ttk.LabelFrame(frame_left, text="PID Controller", padding=10)
frame_pid.pack(fill=tk.X, pady=5)

ttk.Label(frame_pid, text="Kp:").grid(row=0, column=0, sticky=tk.W)
scale_Kp = ttk.Spinbox(frame_pid, from_=0.1, to=1000, increment=0.1, format="%.1f", width=8, command=update_plots)
scale_Kp.grid(row=0, column=1)

ttk.Label(frame_pid, text="Ki:").grid(row=1, column=0, sticky=tk.W)
scale_Ki = ttk.Spinbox(frame_pid, from_=0.1, to=1000, increment=0.1, format="%.1f", width=8, command=update_plots)
scale_Ki.grid(row=1, column=1)

ttk.Label(frame_pid, text="Kd:").grid(row=2, column=0, sticky=tk.W)
scale_Kd = ttk.Spinbox(frame_pid, from_=0.1, to=1000, increment=0.1, format="%.1f", width=8, command=update_plots)
scale_Kd.grid(row=2, column=1)

# القيم المحسوبة
frame_metrics = ttk.LabelFrame(frame_left, text="Response Metrics", padding=10)
frame_metrics.pack(fill=tk.X, pady=5)

label_max_overshoot_before = ttk.Label(frame_metrics, text="Max Overshoot Before: N/A")
label_max_overshoot_before.pack(anchor=tk.W)
label_settling_time_before = ttk.Label(frame_metrics, text="Settling Time Before: N/A")
label_settling_time_before.pack(anchor=tk.W)
label_rise_time_before = ttk.Label(frame_metrics, text="Rise Time Before: N/A")
label_rise_time_before.pack(anchor=tk.W)
label_peak_time_before = ttk.Label(frame_metrics, text="Peak Time Before: N/A")
label_peak_time_before.pack(anchor=tk.W)
label_steady_state_error_before = ttk.Label(frame_metrics, text="Steady-State Error Before: N/A")
label_steady_state_error_before.pack(anchor=tk.W)

label_max_overshoot_after = ttk.Label(frame_metrics, text="Max Overshoot After: N/A")
label_max_overshoot_after.pack(anchor=tk.W)
label_settling_time_after = ttk.Label(frame_metrics, text="Settling Time After: N/A")
label_settling_time_after.pack(anchor=tk.W)
label_rise_time_after = ttk.Label(frame_metrics, text="Rise Time After: N/A")
label_rise_time_after.pack(anchor=tk.W)
label_peak_time_after = ttk.Label(frame_metrics, text="Peak Time After: N/A")
label_peak_time_after.pack(anchor=tk.W)
label_steady_state_error_after = ttk.Label(frame_metrics, text="Steady-State Error After: N/A")
label_steady_state_error_after.pack(anchor=tk.W)

# رسائل الخطأ
label_error = ttk.Label(frame_left, text="", foreground="red")
label_error.pack(anchor=tk.W, pady=5)

# زر طباعة النتائج
button_print_values = ttk.Button(frame_left, text="Print Values")
button_print_values.pack(fill=tk.X, pady=5)

# إضافة نص الحقوق أسفل النافذة
label_footer = ttk.Label(
    root,
    text="""All rights reserved - University of Anbar - 2026
Eng Qusay  Alsultan""",
    font=("Arial", 8, "italic"),
    anchor="w"
)

label_footer.place(x=10, y=660)

######################################################################

# حساب استجابة الخطوة
def calculate_step_response(num, den):
    system = signal.TransferFunction(num, den)
    t, y = signal.step(system)
    return t, y

# حساب المقاييس (Max Overshoot, Settling Time, Rise Time)
def calculate_response_metrics2(t, y):
    max_overshoot = (np.max(y) - 1) * 100 if np.max(y) > 1 else 0
    settling_time = None
    rise_time = None

    for i, value in enumerate(y):
        if rise_time is None and value >= 0.9:
            rise_time = t[i]
        if value < 0.98 or value > 1.02:
            settling_time = t[i]
    return max_overshoot, settling_time, rise_time
    ##############################################################
 # طباعة القيم المحسوبة في نافذة جديدة على شكل جدول
def print_calculated_values2(max_overshoot_before, settling_time_before, rise_time_before, peak_time_before, steady_state_error_before,
                            max_overshoot_after, settling_time_after, rise_time_after, peak_time_after, steady_state_error_after):
    new_window = tk.Toplevel()
    new_window.title("Calculated Values")
    new_window.geometry("800x400")

    # إنشاء الجدول
    columns = ("Metric", "Before PID", "After PID", "Judgment")
    table = ttk.Treeview(new_window, columns=columns, show="headings", height=6)
    table.pack(fill=tk.BOTH, expand=True)

    # إعداد رؤوس الجدول
    for col in columns:
        table.heading(col, text=col)
        table.column(col, anchor=tk.CENTER)

    # إدخال البيانات
    def judge_overshoot(value):
        return "Good" if value < 10 else "Needs Improvement"

    def judge_settling_time(value):
        return "Good" if value and value < 5 else "Needs Improvement"

    def judge_steady_state_error(value):
        return "Good" if value < 2 else "Needs Improvement"

    def judge_rise_time(value):
        return "Good" if value and value < 3 else "Needs Improvement"

    def judge_peak_time(value):
        return "Good" if value and value < 2 else "Needs Improvement"

    table.insert("", tk.END, values=("Max Overshoot (%)", f"{max_overshoot_before:.2f}", f"{max_overshoot_after:.2f}", 
                                 f"{judge_overshoot(max_overshoot_after)}"))
    table.insert("", tk.END, values=("Settling Time (s)", 
                                 f"{settling_time_before:.2f}" if settling_time_before else "N/A", 
                                 f"{settling_time_after:.2f}" if settling_time_after else "N/A", 
                                 f"{judge_settling_time(settling_time_after)}"))
    table.insert("", tk.END, values=("Rise Time (s)", 
                                 f"{rise_time_before:.2f}" if rise_time_before else "N/A", 
                                 f"{rise_time_after:.2f}" if rise_time_after else "N/A", 
                                 f"{judge_rise_time(rise_time_after)}"))
    table.insert("", tk.END, values=("Peak Time (s)", 
                                 f"{peak_time_before:.2f}" if peak_time_before else "N/A", 
                                 f"{peak_time_after:.2f}" if peak_time_after else "N/A", 
                                 f"{judge_peak_time(peak_time_after)}"))
    table.insert("", tk.END, values=("Steady-State Error (%)", 
                                 f"{steady_state_error_before:.2f}", 
                                 f"{steady_state_error_after:.2f}", 
                                 f"{judge_steady_state_error(steady_state_error_after)}"))

    # زر إغلاق النافذة
    ttk.Button(new_window, text="Close", command=new_window.destroy).pack(pady=10)
###################################################################
# فتح نافذة I-PD
def open_ipd_window1():
    def update_ipd_plots1(*args):
        try:
            num = [float(i) for i in entry_num.get().split(',')]
            den = [float(i) for i in entry_den.get().split(',')]

            system = ctl.TransferFunction(num, den)
            t_before_M1, y_before_M1 = ctl.step_response(system)
            step_info1 = ctl.step_info(system)  # استخدام step_info لاستخراج القيم مباشرة
        
            max_overshoot_M1 = step_info1.get('Overshoot', 0)
            settling_time_M1 = step_info1.get('SettlingTime', None)
            rise_time_M1 = step_info1.get('RiseTime', None)
            peak_time_M1 = step_info1.get('PeakTime', None)
            steady_state_error_M1 = abs(1 - y_before_M1[-1]) * 100  # نسبة مئوية

            Kp_va2 = float(scale_ipd_Kp2.get().strip())
            Ki_va2 =  float(scale_ipd_Ki2.get().strip())
            Kd_va2 = float(scale_ipd_Kd2.get().strip())

            G_p = ctl.TransferFunction(num, den)  # تمثيل النظام باستخدام دالة التحويل

        # حساب البسط
            numerator = np.polymul([Ki_va2], G_p.num[0][0])  # البسط: Ki * num

        # حساب المقام
        # نضرب s في المقام الخاص بالبلانت (den)
            s_times_den = np.polymul([1, 0], G_p.den[0][0])  # s * den

        # حساب الحدود الأخرى في المقام
            term1 = np.polymul([Ki_va2], G_p.num[0][0])  # Ki * G_p(s)
            term2 = np.polymul([Kp_va2, 0], G_p.num[0][0])  # Kp * s * G_p(s)
            term3 = np.polymul([Kd_va2, 0, 0], G_p.num[0][0])  # Kd * s^2 * G_p(s)

        # جمع جميع حدود المقام
            denominator = np.polyadd(s_times_den, term1)
            denominator = np.polyadd(denominator, term2)
            denominator = np.polyadd(denominator, term3)

        # تعريف النظام المغلق
            T2 = ctl.TransferFunction(numerator, denominator)  # النظام المغلق
            t_after2, y_after2 = ctl.step_response(T2)

            step_info_T2 = ctl.step_info(T2)
            max_overshoot_M2 = step_info_T2.get('Overshoot', 0)
            settling_time_M2 = step_info_T2.get('SettlingTime', None)
            rise_time_M2 = step_info_T2.get('RiseTime', None)
            peak_time_M2 = step_info_T2.get('PeakTime', None)
            steady_state_error_M2 = abs(1 - y_after2[-1]) * 100  # نسبة مئوية
            # تحديث الرسومات
            ax_ipd1.clear()
            ax_ipd1.plot(t_before_M1, y_before_M1, label="Before I-PD Control", color="blue")
            ax_ipd1.set_title("Step Response Before I-PD Control")
            ax_ipd1.set_xlabel("Time (s)")
            ax_ipd1.set_ylabel("Response")
            ax_ipd1.legend()
            ax_ipd1.grid(True)
            ax_ipd1.grid(True)
            ax_ipd2.clear()
            ax_ipd2.plot(t_after2, y_after2, label="After I-PD Control", color="red")
            ax_ipd2.set_title("Step Response After I-PD Control")
            ax_ipd2.set_xlabel("Time (s)")
            ax_ipd2.set_ylabel("Response")
            ax_ipd2.legend()
            ax_ipd2.grid(True)
            ipd_canvas.draw()

            # تحديث القيم
            label_max_overshoot_before.config(text=f"Max Overshoot Before: {max_overshoot_M1:.2f}%")
            label_settling_time_before.config(text=f"Settling Time Before: {settling_time_M1:.2f}s" if settling_time_M1 else "N/A")
            label_rise_time_before.config(text=f"Rise Time Before: {rise_time_M1:.2f}s" if rise_time_M1 else "N/A")
            label_peak_time_before.config(text=f"Peak Time Before: {peak_time_M1:.2f}s" if peak_time_M1 else "N/A")
            label_steady_state_error_before.config(text=f"Steady-State Error Before: {steady_state_error_M1:.2f}%")

            label_max_overshoot_after.config(text=f"Max Overshoot After: {max_overshoot_M2:.2f}%")
            label_settling_time_after.config(text=f"Settling Time After: {settling_time_M2:.2f}s" if settling_time_M2 else "N/A")
            label_rise_time_after.config(text=f"Rise Time After: {rise_time_M2:.2f}s" if rise_time_M2 else "N/A")
            label_peak_time_after.config(text=f"Peak Time After: {peak_time_M2:.2f}s" if peak_time_M2 else "N/A")
            label_steady_state_error_after.config(text=f"Steady-State Error After: {steady_state_error_M2:.2f}%")
###########################################################################################
    

            button_print_values.config(command=lambda: print_calculated_values2(
                max_overshoot_M1, settling_time_M1, rise_time_M1, peak_time_M1, steady_state_error_M1,
                max_overshoot_M2, settling_time_M2, rise_time_M2, peak_time_M2, steady_state_error_M2
            ))

        
#########################################################################################################
        except Exception as e:
            label_ipd_error.config(text=f"Error: {e}")

    # إنشاء نافذة جديدة
    ipd_window = tk.Toplevel(root)
    ipd_window.title("I-PD Controller")
    ipd_window.geometry("1200x800")

    # إعداد الرسم البياني
    fig_ipd, (ax_ipd1, ax_ipd2) = plt.subplots(2, 1, figsize=(6, 8))
    fig_ipd.tight_layout(pad=3.0)
    ax_ipd1.grid(True)
    ax_ipd2.grid(True)

    # إطار الرسومات
    ipd_canvas = FigureCanvasTkAgg(fig_ipd, master=ipd_window)
    ipd_canvas.get_tk_widget().pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

    # إطار الإدخال
    frame_ipd_left = ttk.Frame(ipd_window, width=400, padding=10)
    frame_ipd_left.pack(side=tk.LEFT, fill=tk.BOTH)

    frame_ipd_pid = ttk.LabelFrame(frame_ipd_left, text="I-PD Controller", padding=10)
    frame_ipd_pid.pack(fill=tk.X, pady=5)

    # السلايدرات
    ttk.Label(frame_ipd_pid, text="Kp:").grid(row=0, column=0, sticky=tk.W)
    scale_ipd_Kp2 = tk.Spinbox(frame_ipd_pid, from_=0.1, to=1000, increment=0.1, format="%.1f", width=8, command=update_ipd_plots1)
    scale_ipd_Kp2.grid(row=0, column=1)
 

    ttk.Label(frame_ipd_pid, text="Ki:").grid(row=1, column=0, sticky=tk.W)
    scale_ipd_Ki2 = tk.Spinbox(frame_ipd_pid, from_=0.1, to=1000, increment=0.1, format="%.1f", width=8, command=update_ipd_plots1)
    scale_ipd_Ki2.grid(row=1, column=1)
    

    ttk.Label(frame_ipd_pid, text="Kd:").grid(row=2, column=0, sticky=tk.W)
    scale_ipd_Kd2 = tk.Spinbox(frame_ipd_pid, from_=0.1, to=1000, increment=0.1, format="%.1f", width=8, command=update_ipd_plots1)
    scale_ipd_Kd2.grid(row=2, column=1)
    

    # القيم المحسوبة
    frame_ipd_metrics = ttk.LabelFrame(frame_ipd_left, text="Response Metrics", padding=10)
    frame_ipd_metrics.pack(fill=tk.X, pady=5)

    label_max_overshoot_before = ttk.Label(frame_ipd_metrics, text="Max Overshoot Before: N/A")
    label_max_overshoot_before.pack(anchor=tk.W)
    label_settling_time_before = ttk.Label(frame_ipd_metrics, text="Settling Time Before: N/A")
    label_settling_time_before.pack(anchor=tk.W)
    label_rise_time_before = ttk.Label(frame_ipd_metrics, text="Rise Time Before: N/A")
    label_rise_time_before.pack(anchor=tk.W)
    label_peak_time_before = ttk.Label(frame_ipd_metrics, text="Peak Time Before: N/A")
    label_peak_time_before.pack(anchor=tk.W)
    label_steady_state_error_before = ttk.Label(frame_ipd_metrics, text="Steady-State Error Before: N/A")
    label_steady_state_error_before.pack(anchor=tk.W)

    label_max_overshoot_after = ttk.Label(frame_ipd_metrics, text="Max Overshoot After: N/A")
    label_max_overshoot_after.pack(anchor=tk.W)
    label_settling_time_after = ttk.Label(frame_ipd_metrics, text="Settling Time After: N/A")
    label_settling_time_after.pack(anchor=tk.W)
    label_rise_time_after = ttk.Label(frame_ipd_metrics, text="Rise Time After: N/A")
    label_rise_time_after.pack(anchor=tk.W)
    label_peak_time_after = ttk.Label(frame_ipd_metrics, text="Peak Time After: N/A")
    label_peak_time_after.pack(anchor=tk.W)
    label_steady_state_error_after = ttk.Label(frame_ipd_metrics, text="Steady-State Error After: N/A")
    label_steady_state_error_after.pack(anchor=tk.W)
    
    

    # رسائل الخطأ
    label_ipd_error = ttk.Label(frame_ipd_left, text="", foreground="red")
    label_ipd_error.pack(anchor=tk.W, pady=5)
    
    # زر طباعة النتائج
    button_print_values = ttk.Button(frame_ipd_left, text="Print Values")
    button_print_values.pack(fill=tk.X, pady=5)
    
button_open_ipd = ttk.Button(frame_left, text="Open I-PD Control Window", command=open_ipd_window1)
button_open_ipd.pack(pady=10)
##############################################pi-d#######################################################################
# حساب استجابة الخطوة
def calculate_step_response3(num, den):
    system = signal.TransferFunction(num, den)
    t, y = signal.step(system)
    return t, y

# حساب المقاييس (Max Overshoot, Settling Time, Rise Time)
def calculate_response_metrics2(t, y):
    max_overshoot = (np.max(y) - 1) * 100 if np.max(y) > 1 else 0
    settling_time = None
    rise_time = None

    for i, value in enumerate(y):
        if rise_time is None and value >= 0.9:
            rise_time = t[i]
        if value < 0.98 or value > 1.02:
            settling_time = t[i]
    return max_overshoot, settling_time, rise_time
    ##############################################################
 # طباعة القيم المحسوبة في نافذة جديدة على شكل جدول
def print_calculated_values3(max_overshoot_before, settling_time_before, rise_time_before, peak_time_before, steady_state_error_before,
                            max_overshoot_after, settling_time_after, rise_time_after, peak_time_after, steady_state_error_after):
    new_window = tk.Toplevel()
    new_window.title("Calculated Values")
    new_window.geometry("800x400")

    # إنشاء الجدول
    columns = ("Metric", "Before PID", "After PID", "Judgment")
    table = ttk.Treeview(new_window, columns=columns, show="headings", height=6)
    table.pack(fill=tk.BOTH, expand=True)

    # إعداد رؤوس الجدول
    for col in columns:
        table.heading(col, text=col)
        table.column(col, anchor=tk.CENTER)

    # إدخال البيانات
    def judge_overshoot(value):
        return "Good" if value < 10 else "Needs Improvement"

    def judge_settling_time(value):
        return "Good" if value and value < 5 else "Needs Improvement"

    def judge_steady_state_error(value):
        return "Good" if value < 2 else "Needs Improvement"

    def judge_rise_time(value):
        return "Good" if value and value < 3 else "Needs Improvement"

    def judge_peak_time(value):
        return "Good" if value and value < 2 else "Needs Improvement"

    table.insert("", tk.END, values=("Max Overshoot (%)", f"{max_overshoot_before:.2f}", f"{max_overshoot_after:.2f}", 
                                 f"{judge_overshoot(max_overshoot_after)}"))
    table.insert("", tk.END, values=("Settling Time (s)", 
                                 f"{settling_time_before:.2f}" if settling_time_before else "N/A", 
                                 f"{settling_time_after:.2f}" if settling_time_after else "N/A", 
                                 f"{judge_settling_time(settling_time_after)}"))
    table.insert("", tk.END, values=("Rise Time (s)", 
                                 f"{rise_time_before:.2f}" if rise_time_before else "N/A", 
                                 f"{rise_time_after:.2f}" if rise_time_after else "N/A", 
                                 f"{judge_rise_time(rise_time_after)}"))
    table.insert("", tk.END, values=("Peak Time (s)", 
                                 f"{peak_time_before:.2f}" if peak_time_before else "N/A", 
                                 f"{peak_time_after:.2f}" if peak_time_after else "N/A", 
                                 f"{judge_peak_time(peak_time_after)}"))
    table.insert("", tk.END, values=("Steady-State Error (%)", 
                                 f"{steady_state_error_before:.2f}", 
                                 f"{steady_state_error_after:.2f}", 
                                 f"{judge_steady_state_error(steady_state_error_after)}"))

    # زر إغلاق النافذة
    ttk.Button(new_window, text="Close", command=new_window.destroy).pack(pady=10)
###################################################################
# فتح نافذة pi-d
def open_ipd_window():
    def update_ipd_plots(*args):
        try:
            num = [float(i) for i in entry_num.get().split(',')]
            den = [float(i) for i in entry_den.get().split(',')]

            system = ctl.TransferFunction(num, den)
            t_before_q1, y_before_q1 = ctl.step_response(system)
            step_info3 = ctl.step_info(system)  # استخدام step_info لاستخراج القيم مباشرة
        
            max_overshoot_q1 = step_info3.get('Overshoot', 0)
            settling_time_q1 = step_info3.get('SettlingTime', None)
            rise_time_q1 = step_info3.get('RiseTime', None)
            peak_time_q1 = step_info3.get('PeakTime', None)
            steady_state_error_q1 = abs(1 - y_before_q1[-1]) * 100  # نسبة مئوية

            Kp_val3 = float(scale_ipd_Kp3.get().strip())
            Ki_val3 =  float(scale_ipd_Ki3.get().strip())
            Kd_val3 = float(scale_ipd_Kd3.get().strip())

            # تمثيل النظام باستخدام دالة التحويل
            G_p = ctl.TransferFunction(num, den)

            # حساب البسط
            numerator_pid = np.polymul([Kp_val3, Ki_val3], G_p.num[0][0])  # البسط: (Kp*s + Ki) * num

            # حساب المقام
            denominator_terms = [
                np.polymul([1, 0], G_p.den[0][0]),  # s * den
                np.polymul([Kp_val3, Ki_val3], G_p.num[0][0]),  # (Kp*s + Ki) * num
                np.polymul([Kd_val3, 0, 0], G_p.num[0][0])  # Kd * s^2 * num
            ]

            # جمع جميع حدود المقام
            denominator = np.polyadd(denominator_terms[0], denominator_terms[1])
            denominator = np.polyadd(denominator, denominator_terms[2])

            # تعريف النظام المغلق
            T3 = ctl.TransferFunction(numerator_pid, denominator)  # النظام المغلق

            # حساب استجابة الخطوة
            t_after_q2, y_after_q2 = ctl.step_response(T3)

            step_info_T3 = ctl.step_info(T3)
            max_overshoot_q2 = step_info_T3.get('Overshoot', 0)
            settling_time_q2 = step_info_T3.get('SettlingTime', None)
            rise_time_q2 = step_info_T3.get('RiseTime', None)
            peak_time_q2 = step_info_T3.get('PeakTime', None)
            steady_state_error_q2 = abs(1 - y_after_q2[-1]) * 100  # نسبة مئوية

            # تحديث الرسومات
            ax_ipd1.clear()
            ax_ipd1.plot(t_before_q1, y_before_q1, label="Before PI-D Control", color="blue")
            ax_ipd1.set_title("Step Response Before PI-D Control")
            ax_ipd1.set_xlabel("Time (s)")
            ax_ipd1.set_ylabel("Response")
            ax_ipd1.legend()
            ax_ipd1.grid(True)
            ax_ipd2.clear()
            ax_ipd2.plot(t_after_q2, y_after_q2, label="After PI-D Control", color="red")
            ax_ipd2.set_title("Step Response After PI-D Control")
            ax_ipd2.set_xlabel("Time (s)")
            ax_ipd2.set_ylabel("Response")
            ax_ipd2.legend()
            ax_ipd2.grid(True)
            ipd_canvas.draw()

            # تحديث القيم
            label_max_overshoot_before.config(text=f"Max Overshoot Before: {max_overshoot_q1:.2f}%")
            label_settling_time_before.config(text=f"Settling Time Before: {settling_time_q1:.2f}s" if settling_time_q1 else "N/A")
            label_rise_time_before.config(text=f"Rise Time Before: {rise_time_q1:.2f}s" if rise_time_q1 else "N/A")
            label_peak_time_before.config(text=f"Peak Time Before: {peak_time_q1:.2f}s" if peak_time_q1 else "N/A")
            label_steady_state_error_before.config(text=f"Steady-State Error Before: {steady_state_error_q1:.2f}%")

            label_max_overshoot_after.config(text=f"Max Overshoot After: {max_overshoot_q2:.2f}%")
            label_settling_time_after.config(text=f"Settling Time After: {settling_time_q2:.2f}s" if settling_time_q2 else "N/A")
            label_rise_time_after.config(text=f"Rise Time After: {rise_time_q2:.2f}s" if rise_time_q2 else "N/A")
            label_peak_time_after.config(text=f"Peak Time After: {peak_time_q2:.2f}s" if peak_time_q2 else "N/A")
            label_steady_state_error_after.config(text=f"Steady-State Error After: {steady_state_error_q2:.2f}%")

###########################################################################################
    

            # ربط القيم بطباعة النتائج
            button_print_values.config(command=lambda: print_calculated_values3(
                max_overshoot_q1, settling_time_q1, rise_time_q1, peak_time_q1, steady_state_error_q1,
                max_overshoot_q2, settling_time_q2, rise_time_q2, peak_time_q2, steady_state_error_q2
            ))

        
#########################################################################################################
        except Exception as e:
            label_ipd_error.config(text=f"Error: {e}")

    # إنشاء نافذة جديدة
    ipd_window = tk.Toplevel(root)
    ipd_window.title("PI-D Controller")
    ipd_window.geometry("1200x800")

    # إعداد الرسم البياني
    fig_ipd, (ax_ipd1, ax_ipd2) = plt.subplots(2, 1, figsize=(6, 8))
    fig_ipd.tight_layout(pad=3.0)
    ax_ipd1.grid(True)
    ax_ipd2.grid(True)

    # إطار الرسومات
    ipd_canvas = FigureCanvasTkAgg(fig_ipd, master=ipd_window)
    ipd_canvas.get_tk_widget().pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

    # إطار الإدخال
    frame_ipd_left = ttk.Frame(ipd_window, width=400, padding=10)
    frame_ipd_left.pack(side=tk.LEFT, fill=tk.BOTH)

    frame_ipd_pid = ttk.LabelFrame(frame_ipd_left, text="PI-D Controller", padding=10)
    frame_ipd_pid.pack(fill=tk.X, pady=5)

    # السلايدرات
    ttk.Label(frame_ipd_pid, text="Kp:").grid(row=0, column=0, sticky=tk.W)
    scale_ipd_Kp3 = tk.Spinbox(frame_ipd_pid, from_=0.1, to=1000, increment=0.1, format="%.1f", width=8, command=update_ipd_plots)
    scale_ipd_Kp3.grid(row=0, column=1)
    

    ttk.Label(frame_ipd_pid, text="Ki:").grid(row=1, column=0, sticky=tk.W)
    scale_ipd_Ki3 = tk.Spinbox(frame_ipd_pid, from_=0.1, to=1000, increment=0.1, format="%.1f", width=8, command=update_ipd_plots)
    scale_ipd_Ki3.grid(row=1, column=1)


    ttk.Label(frame_ipd_pid, text="Kd:").grid(row=2, column=0, sticky=tk.W)
    scale_ipd_Kd3 = tk.Spinbox(frame_ipd_pid, from_=0.1, to=1000, increment=0.1, format="%.1f", width=8, command=update_ipd_plots)
    scale_ipd_Kd3.grid(row=2, column=1)
    

    # القيم المحسوبة
    frame_ipd_metrics3 = ttk.LabelFrame(frame_ipd_left, text="Response Metrics", padding=10)
    frame_ipd_metrics3.pack(fill=tk.X, pady=5)

    label_max_overshoot_before = ttk.Label(frame_ipd_metrics3, text="Max Overshoot Before: N/A")
    label_max_overshoot_before.pack(anchor=tk.W)
    label_settling_time_before = ttk.Label(frame_ipd_metrics3, text="Settling Time Before: N/A")
    label_settling_time_before.pack(anchor=tk.W)
    label_rise_time_before = ttk.Label(frame_ipd_metrics3, text="Rise Time Before: N/A")
    label_rise_time_before.pack(anchor=tk.W)
    label_peak_time_before = ttk.Label(frame_ipd_metrics3, text="Peak Time Before: N/A")
    label_peak_time_before.pack(anchor=tk.W)
    label_steady_state_error_before = ttk.Label(frame_ipd_metrics3, text="Steady-State Error Before: N/A")
    label_steady_state_error_before.pack(anchor=tk.W)

    label_max_overshoot_after = ttk.Label(frame_ipd_metrics3, text="Max Overshoot After: N/A")
    label_max_overshoot_after.pack(anchor=tk.W)
    label_settling_time_after = ttk.Label(frame_ipd_metrics3, text="Settling Time After: N/A")
    label_settling_time_after.pack(anchor=tk.W)
    label_rise_time_after = ttk.Label(frame_ipd_metrics3, text="Rise Time After: N/A")
    label_rise_time_after.pack(anchor=tk.W)
    label_peak_time_after = ttk.Label(frame_ipd_metrics3, text="Peak Time After: N/A")
    label_peak_time_after.pack(anchor=tk.W)
    label_steady_state_error_after = ttk.Label(frame_ipd_metrics3, text="Steady-State Error After: N/A")
    label_steady_state_error_after.pack(anchor=tk.W)
    
    
    

    # رسائل الخطأ
    label_ipd_error = ttk.Label(frame_ipd_left, text="", foreground="red")
    label_ipd_error.pack(anchor=tk.W, pady=5)
    
    # زر طباعة النتائج
    button_print_values = ttk.Button(frame_ipd_left, text="Print Values")
    button_print_values.pack(fill=tk.X, pady=5)
    
button_open_ipd = ttk.Button(frame_left, text="Open PI-D Control Window", command=open_ipd_window)
button_open_ipd.pack(pady=10)

# تشغيل التطبيق
root.mainloop()
