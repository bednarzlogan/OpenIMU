import requests

# Define the URL of the Flask server
url = "http://127.0.0.1:5000/dump_matrix"  # Replace with your server's IP if necessary


def publish_matrix(sympy_matrix, label):
    # Convert matrix elements to strings for JSON serialization
    matrix_data = [[str(element) for element in row] for row in sympy_matrix.tolist()]

    # Set up the data payload with matrix and label
    payload = {
        "matrix": matrix_data,
        "label": label  # "Perturbation Q_be_inv"
    }

    # Send POST request to the Flask API
    response = requests.post(url, json=payload)

    # Check if the request was successful
    if response.status_code == 200:
        print("Matrix sent successfully!")
    else:
        print("Failed to send matrix:", response.text)
