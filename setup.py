from setuptools import setup, find_packages

setup(
    name="fimop",
    version="0.1.0",
    description="F-IMOP: Filtered-Error Inertia-Metric Optimal Projection for Safe Robot Control",
    author="Haozhe Jia",
    author_email="your-email@example.com",
    url="https://github.com/your-username/fimop",
    packages=find_packages(),
    python_requires=">=3.8",
    install_requires=[
        "numpy>=1.20.0",
        "matplotlib>=3.4.0",
    ],
    extras_require={
        "dev": [
            "pytest>=6.0",
        ],
    },
    classifiers=[
        "Development Status :: 3 - Alpha",
        "Intended Audience :: Science/Research",
        "License :: OSI Approved :: MIT License",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Topic :: Scientific/Engineering :: Artificial Intelligence",
    ],
)
