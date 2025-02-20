from flask import Flask, request, jsonify, render_template_string, redirect, url_for
from sympy import Matrix
import webbrowser
import threading

app = Flask(__name__)
matrix_bank = []


def bank_matrix(matrix, label="Matrix"):
    matrix_bank.append((matrix, label))


@app.route('/')
def show_matrices():
    html_template = """
    <html>
    <head>
        <style>
            body { background-color: #121212; color: #e0e0e0; font-family: Courier, monospace; }
            .matrix { margin-bottom: 20px; }
            h2 { color: #bb86fc; font-size: 24px; }
            pre { font-size: 18px; line-height: 1.5; }
            .divider { border-top: 1px solid #555; margin-top: 15px; margin-bottom: 15px; }
            .button { background-color: #bb86fc;
                      color: white;
                      padding: 10px 20px;
                      border: none;
                      cursor: pointer;
                      font-size: 16px; }
        </style>
        <script>
            setTimeout(function(){
                window.location.reload(1);
            }, 5000);
        </script>
    </head>
    <body>
        <h1>Matrix Bank</h1>
        <button class="button" onclick="window.location.href='/clear'">Clear Screen</button>
        <br><br>
        {% for matrix, label in matrices %}
            <div class="matrix">
                <h2>{{ label }}</h2>
                <pre>
                {% for row in matrix %}
                    {{ row|join(", ") }}
                {% endfor %}
                </pre>
                <div class="divider"></div>
            </div>
        {% endfor %}
    </body>
    </html>
    """

    formatted_matrices = []
    for matrix, label in matrix_bank:
        matrix_str_elements = [[str(element) for element in row] for row in matrix.tolist()]
        column_widths = [max(len(row[i]) for row in matrix_str_elements) for i in range(matrix.cols)]
        formatted_rows = [
            ["{:>{width}},".format(element, width=column_widths[i]) for i, element in enumerate(row)]
            for row in matrix_str_elements
        ]
        # Remove the extra comma in the last element of each row
        formatted_rows = [[element.rstrip(",") for element in row] for row in formatted_rows]
        formatted_matrices.append((formatted_rows, label))

    return render_template_string(html_template, matrices=formatted_matrices)


@app.route('/dump_matrix', methods=['POST'])
def dump_matrix():
    data = request.get_json()
    if 'matrix' not in data or 'label' not in data:
        return jsonify({"error": "Invalid input format. Expected 'matrix' and 'label'."}), 400

    matrix_data = Matrix(data['matrix'])
    label = data['label']
    bank_matrix(matrix_data, label)

    return jsonify({"status": "Matrix added successfully"}), 200


@app.route('/clear')
def clear_matrices():
    matrix_bank.clear()  # Clear the matrix bank
    return redirect(url_for('show_matrices'))  # Redirect back to the main page


def run_server():
    app.run(host="0.0.0.0", port=5000, debug=False)


if __name__ == "__main__":
    # Start the server in a separate thread
    threading.Thread(target=run_server).start()

    # Open the web page in the default browser
    webbrowser.open("http://localhost:5000")
