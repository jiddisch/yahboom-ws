# Yahboom ROSMASTER M1 - מדריך סימולציה והתקנה

מסמך זה מאגד את כל הפקודות החיוניות לעבודה עם הרובוט (M1) על מחשב הפיתוח (Ubuntu/Raspberry Pi).

## 1. הכנת הסביבה (צעד ראשון חובה)

יש להריץ את שתי הפקודות האלו בכל פעם שפותחים טרמינל חדש.

### א. הפעלת הסביבה הוירטואלית של Python

מבודדת את ספריות הפייתון של הפרויקט מהמערכת הראשית כדי למנוע התנגשויות.

```bash
source venv/bin/activate
```

### ב. טעינת סביבת העבודה של ROS2

גורמת למערכת ההפעלה "להכיר" את החבילות והקוד שלנו (אחרי שבנינו אותם).

```bash
source install/setup.bash
```

---

## 2. בנייה והתקנה (Build & Install)

### תיקון שגיאות בנייה (פעולה חד-פעמית)

פקודה זו יוצרת קובץ שמונע מ-`colcon` לנסות לבנות את הספריות הפנימיות של ה-venv (מונע את השגיאות האדומות בבנייה).

```bash
touch venv/COLCON_IGNORE
```

### התקנת תלויות לסימולציה

התקנת הכלים הדרושים כדי להריץ את Gazebo ולקרוא קבצי רובוט מורכבים (Xacro).

```bash
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-ros-core ros-humble-geometry2 ros-humble-xacro ros-humble-teleop-twist-keyboard
```

### בניית הפרויקט

הופך את הקוד (Python/C++) לתוכנה שניתן להריץ. הדגל `--packages-select` חוסך זמן ובונה רק את החבילה הרלוונטית.

```bash
colcon build --packages-select yahboomcar_description
```

_(לאחר כל בנייה מומלץ להריץ שוב `source install/setup.bash`)_

---

## 3. ויזואליזציה וסימולציה

### אפשרות א': צפייה במודל בלבד (RViz)

פותח את **RViz**. מציג את הרובוט כ"פסל" סטטי.

- **מטרה:** בדיקה שכל החלקים (גלגלים, מצלמה, לידאר) מחוברים במקום הנכון.

```bash
ros2 launch yahboomcar_description display_M1.launch.py
```

### אפשרות ב': סימולציה פיזיקלית (Gazebo)

פותח את **Gazebo**. כאן לרובוט יש פיזיקה, והוא יכול לנסוע במרחב.

**1. הגדרת נתיבים קבועה (חובה לביצוע פעם אחת):**
כדי ש-Gazebo ימצא את המודלים הגרפיים (ולא יציג רובוט שקוף), יש להוסיף את נתיב הפרויקט לקובץ ההגדרות.
הרץ פקודה זו פעם אחת בטרמינל:

```bash
echo "export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:$HOME/git/yahboom-ws/src" >> ~/.bashrc
source ~/.bashrc
source /usr/share/gazebo/setup.sh
```

**2. הרצת הסימולציה:**

```bash
ros2 launch yahboomcar_description simulate_M1.launch.py
```

### אפשרות ג': שליטה ברובוט (מקלדת)

יש לפתוח **טרמינל חדש** (לא לשכוח לעשות `source` כמו בשלב 1) ולהריץ:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**מקשים לשליטה:**

- `i` = קדימה | `,` = אחורה
- `j` = סיבוב שמאלה | `l` = סיבוב ימינה
- `Shift + j` = נסיעה הצידה שמאלה (Strafe)
- `Shift + l` = נסיעה הצידה ימינה (Strafe)
- `k` = עצירה
- `q/z` = שינוי מהירות

---

## 4. פקודות ניהול ופיתוח ושיתוף עם AI

### Git - יצירת ענף עבודה

מומלץ תמיד לעבוד על ענף נפרד כדי לשמור על הקוד הראשי נקי.

```bash
git switch -c simulation-setup
```

### חיפוש מהיר בקוד

חיפוש מחרוזת (למשל "gazebo") בכל הקבצים בתיקיית `src`.

```bash
grep -rn "gazebo" src/ | head -n 20
```

### העתקת תוכן קבצים (לצורך שיתוף עם AI)

פקודה זו מעתיקה את תוכן הקבצים ללוח (Clipboard) כדי שניתן יהיה להדביק אותם בצ'אט.

```bash
files="src/path/to/file1.py src/path/to/file2.xml"; sep=""; for f in $files; do printf "%s" "$sep"; awk '1' "$f"; sep="---"; done | xclip -sel c
```

### הצגת מבנה קבצים רלוונטי (לצורך שיתוף עם AI)

פקודה זו מציגה את עץ הקבצים תוך התעלמות מקבצים לא רלוונטיים.

```bash
tree -I "*A1*|*R2*|build|install|log|__pycache__|*.egg-info|site-packages|*.so|*.o|*.a|*.jpg|*.png|*.mp3|*.mp4|*.wav|*.STL|*.stl|*.dae|*.TTF|*.ttf|*.db|*.zip|*.onnx|*.pt|*.pb|*.pbstream|Thumbs.db|venv"
```
